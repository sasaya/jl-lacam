struct LowLevelNode
    who::Vector{Int}
    whither::Vector{Vector{Int}}
    depth::Int
end

function get_child(self::LowLevelNode, who, whither)
    return LowLevelNode(
        vcat(self.who,  [who]),
        filter(x->!all(isempty.(x)) ,vcat(self.whither,[whither])),
        self.depth + 1,
    )
end

mutable struct HighLevelNode
    Q ::Config
    order ::Vector{Int}
    parent ::Union{HighLevelNode, Nothing}
    tree ::Vector{LowLevelNode}
    g::Int
    h::Int
    f::Int
    neighbors::Set{HighLevelNode}

    function HighLevelNode(; 
        Q::Config{Int64}, 
        order, 
        parent::Union{HighLevelNode, Nothing} = nothing, 
        tree::Vector{LowLevelNode} = [LowLevelNode([], [[],], 1)],
        g = 0, 
        h = 0, 
        f = 0, # g + h 
        neighbors = Set()
        )
        new(Q, order, parent, tree, g, h, f, neighbors)
    end

    function HighLevelNode(Q, order, parent, tree, g, h, f, neighbors)
        new(Q, order, parent, tree, g, h, f, neighbors)
    end
end

struct Deadline 
    time_limit_ms::Int
    start_time::DateTime
    function Deadline(time_limit_ms)
        new(time_limit_ms, Dates.now())
    end
end

function elapsed(dl::Deadline)::DateTime
    return (Dates.now() - dl.start_time)
end

function is_expired(dl::Deadline)::Bool
    return (dl |> elapsed |> millisecond) > dl.time_limit_ms
end

mutable struct LaCAMdata
    grid
    starts
    goals
    time_limit_ms
    deadline
    flg_star
    rng
    verbose
    num_agents

    function LaCAMdata(
        grid ::Matrix{Bool}, 
        starts ::Config{<:Integer}, 
        goals  ::Config{<:Integer}, 
        flg_star::Bool = true,
        time_limit_ms::Int = 3000,
        deadline::Deadline = Deadline(time_limit_ms),
        seed::Int = 0,
        verbose::Int = 1
        )
        rng = Random.MersenneTwister(seed)
        num_agents = length(starts)
        new(grid, starts, goals, time_limit_ms, deadline, flg_star, rng, verbose, num_agents)
    end
        function LaCAMdata(;
        grid ::Matrix{Bool}, 
        starts ::Config{<:Integer}, 
        goals  ::Config{<:Integer}, 
        flg_star::Bool = true,
        time_limit_ms::Int = 3000,
        deadline::Deadline = Deadline(time_limit_ms),
        seed::Int = 0,
        verbose::Int = 1
        )
        rng = Random.MersenneTwister(seed)
        num_agents = length(starts)
        new(grid, starts, goals, time_limit_ms, deadline, flg_star, rng, verbose, num_agents)
    end
end



function solve(st::LaCAMdata)
    # println("start solving MAPF")
    dist_tables = [DistTable(st.grid, g) for g in st.goals]
    pibt = PIBT(dist_tables)

    # set search scheme
    OPEN::Vector{HighLevelNode} = []
    # EXPLORED =dict[Config, HighLevelNode] = {}
    N_goal::Union{HighLevelNode, Nothing} = nothing  # ::Union{HighLevelNode, Nothing}

    # set initial node
    Q_init = st.starts
    N_init = HighLevelNode(
        Q=Q_init, order=get_order(st, Q_init, dist_tables), h=get_h_value(st, Q_init, dist_tables)
    )

    push!(OPEN, N_init)
    EXPLORED = Dict(N_init.Q => N_init)

    counter = 0
    # main loop
    while length(OPEN) > 0 && (st.deadline |> is_expired) == false
        N::HighLevelNode = OPEN[begin]
        # @info "new iteration" N.Q
        # goal check
        if isnothing(N_goal) == true && N.Q == st.goals
            N_goal = N
            # st.info(1, f"initial solution found, cost={N_goal.g}")
            # println("initial solution found, cost=$(N_goal.g)")
            @info "initial solution found, cost= $(N_goal.g)"
            # no refinement -> terminate
            if st.flg_star == false
                # println("break")
                break
            end
        end

        # lower bound check
        if isnothing(N_goal) == false && N_goal.g <= N.f
            @debug "lower bound check"
            popfirst!(OPEN)
            continue
        end

        # low-level search end
        if length(N.tree) == 0
            @debug "low-level search end"
            popfirst!(OPEN)
            continue
        end

        # low-level search
        C:: LowLevelNode = popfirst!(N.tree)  # constraints
        if C.depth <= st.num_agents # index python 0, julia 1
            i = N.order[C.depth]
            v = N.Q[i]
            cands = vcat([v], get_neighbors(st.grid, v))
            @debug "get_neighbors" cands
            # shuffle!(st.rng, cands)
            for u in cands
                push!(N.tree, get_child(C, i, u))
                @debug "get_child" get_child(C, i, u)
            end
        end

        # generate the next configuration
        Q_to = configuration_generaotr(st, pibt, N, C)

        if Q_to |> isnothing
            # invalid configuration
            @debug "invalid configuration" st pibt N C
            continue
        elseif Q_to in keys(EXPLORED)
            # known configuration
            @debug "known configuration" Q_to

            N_known = EXPLORED[Q_to]
            push!(N.neighbors, N_known)
            pushfirst!(OPEN, N_known)  # typically helpful
            # rewrite, Dijkstra update
            D = [N]
            while length(D) > 0 && st.flg_star
                N_from = popfirst!(D)
                for N_to in N_from.neighbors
                    g = N_from.g + get_edge_cost(st, N_from.Q, N_to.Q)

                    if g < N_to.g
                        if N_goal |> !isnothing && N_to == N_goal
                            @info "cost update: $(N_goal.g) -> $(g)"
                        end
                        N_to.g = g
                        N_to.f = N_to.g + N_to.h
                        N_to.parent = N_from
                        push!(D, N_to)
                        if N_goal |> !isnothing && N_to.f < N_goal.g
                            pushfirst!(OPEN, N_to)
                        end
                    end
                end
            end
        else
            # new configuration
            @debug "new configuration"

            N_new = HighLevelNode(
                Q=Q_to,
                parent=N,
                order=get_order(st, Q_to, dist_tables),
                g=N.g + get_edge_cost(st, N.Q, Q_to),
                h=get_h_value(st, Q_to, dist_tables),
            )
            push!(N.neighbors, N_new) # 全体的にpushを減らす
            pushfirst!(OPEN, N_new) # 最後尾のPushより遅いので書き換えたい
            EXPLORED[Q_to] = N_new
            @debug "new conf" EXPLORED
        end
    end
    # categorize result
    if N_goal |> !isnothing && length(OPEN) == 0
        # st.info(1, f"reach optimal solution, cost={N_goal.g}")
        @info "reach optimal solution, cost=$(N_goal.g)"
    elseif N_goal |> !isnothing
        # st.info(1, f"suboptimal solution, cost={N_goal.g}")
        @info "suboptimal solution, cost=$(N_goal.g)"
    elseif length(OPEN) == 0
        # st.info(1, "detected unsolvable instance")
        @info "detected unsolvable instance"
    else
        # st.info(1, "failure due to timeout")
        @info "failure due to timeout"
    end

    return backtrack(N_goal)
end

function get_order(self::LaCAMdata, Q::Config{Int}, dist_tables::Vector{DistTable})::Vector{Int}
    # e.g., by descending order of dist(Q[i], g_i)
    # Note that this is not an effective PIBT prioritizationscheme
    order = Vector(Base.OneTo(self.num_agents))
    
    # shuffle!(self.rng, order)
    # println("order ", order)
    tmp = [get(dist_tables[x], Q[x]) for x in order]
    # println("dist_tables get Q ", tmp)
    # # println("dist_tables[3] ", dist_tables[3].table)
    sort!(order, by = x -> get(dist_tables[x], Q[x]), rev=true)
    # println("order ", order)
    return order
end

function get_h_value(self::LaCAMdata, Q::Config, dist_tables::Vector{DistTable})::Int
    # e.g., \sum_i dist(Q[i], g_i)
    cost = 0
    for (agent_idx, loc) in enumerate(Q)
        c = get(dist_tables[agent_idx], loc)
        if c |> isnothing
            return Integer(maxintfloat())
        end
        cost += c
    end
    return cost
end

function backtrack(_N::Union{HighLevelNode, Nothing})::Configs{Int64}
    @debug "backtrac" N
    configs::Configs{Int64} = []
    N = _N
    while (N |> isnothing) == false
        push!(configs, N.Q)
        N = N.parent
    end
    reverse!(configs)
    return configs
end

function configuration_generaotr(self::LaCAMdata, pibt::PIBT,  N::HighLevelNode, C::LowLevelNode)#::Configs

    Q_to::Config = [pibt.NIL_COORD for _ in range(1,self.num_agents)]
    @debug "configuration_generaotr" C

    for k in range(1,C.depth -1) # -1 is needed?
        # println("Q_to", Q_to)
        # println("C.where[k]", C.whither[k])
        # println("C.who[k]", C.who[k])
        Q_to[C.who[k]...] = C.whither[k]
    end
    success = step(pibt, N.Q, Q_to, N.order)
    if success == true
        return Q_to
    else
        return nothing
    end
end

function get_edge_cost(self::LaCAMdata, Q_from::Config{Int}, Q_to::Config{Int})::Int
# e.g., \sum_i | not (Q_from[i] == Q_to[k] == g_i) |
    cost = 0
    for i in range(1,self.num_agents)
        if !(self.goals[i] == Q_from[i] == Q_to[i])
            cost += 1
        end
    end
    return cost
end

# end # module end

# using BenchmarkTools
# @benchmark LaCAM.solve(LaCAM.LaCAMdata(fill(true, 2, 3), [[1, 1], [1, 3]], [[1, 3], [1, 1]], true))