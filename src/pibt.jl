mutable struct DistTable
    const grid::Matrix{Bool}
    const goal::Vector{Int64}
    table::Dict
    Q::Vector{Vector{Int64}}
    
    function DistTable(grid, goal)
        0 in Iterators.flatten(goal) && throw("goal should not include 0")
        Q = [goal]
        # vector型をkeyにすることで'value = table[goal]'と書ける.
        table = Dict([y, x] => length(grid) for y in axes(grid,1), x in axes(grid,2))
        table[goal] = 0
        new(grid, vec(goal), table, Q)
    end
end

# get distance
function Base.get(dt::DistTable, target::AbstractArray{<:Integer})::Integer

    if !isvalid_coordinate(dt.grid, target)
        return length(dt.grid)
    end

    if dt.table[target] < length(dt.table)
        return dt.table[target]
    end
    # pythonだとQはずっと DistTableで保持して
    # 次の計算にも使いまわしているので、同じ仕様にする
    # dt.Q = [dt.goal] # 使いまわさないならこれをつかう
    while length(dt.Q) > 0
        u = popfirst!(dt.Q) # need profile
        d = dt.table[u]
        for v in get_neighbors(dt.grid, u)
            if d + 1 < dt.table[v]
                dt.table[v] = d + 1
                push!(dt.Q, v)
            end
        end
        if u == target
            return d
        end
    end
    return length(dt.grid)
end

mutable struct PIBT
    const grid
    const starts::Config{Int64}
    const goals::Config{Int64}
    const N
    const dist_tables
    const seed
    const NIL
    const NIL_COORD
    occupied_now
    occupied_next
    const rng

    function PIBT(dist_tables::Array{DistTable}, seed=0)
        N = length(dist_tables)
        NIL = N+1; # out of index (length+1)
        grid = dist_tables[begin].grid
        NIL_COORD = [size(grid,1), size(grid,2)]
        occupied_now = Dict([y, x] => NIL for y in axes(grid,1), x in axes(grid,2))
        occupied_next = Dict([y, x] => NIL for y in axes(grid,1), x in axes(grid,2))
        rng = Random.MersenneTwister(seed)

        new(grid, Configs{Int}(undef,0), Configs{Int}(undef,0), N, dist_tables, seed, NIL, NIL_COORD, occupied_now, occupied_next, rng)
    end
end

function funcPIBT(self::PIBT, Q_from::Config, Q_to::Config, i)
    C = vcat([Q_from[i]], get_neighbors(self.grid, Q_from[i]))
    # shuffle!(self.rng, C)  # tie-breaking, randomize
    sort!(C, by = x -> get(self.dist_tables[i], x))

    for v in C
        if self.occupied_next[v] != self.NIL #ここは初期値かどうかなので、pyとjlのindexは関係ない
            continue
        end
        j = self.occupied_now[v]

        if j != self.NIL && Q_to[j] == Q_from[i]
            continue
        end

        Q_to[i] = v
        self.occupied_next[v] = i

        if ( 
            j != self.NIL 
            && Q_to[j] == self.NIL_COORD
            && !funcPIBT(self,Q_from, Q_to, j)
        )
            
            continue
        else
            return true
        end
    end

    Q_to[i] = Q_from[i]
    self.occupied_next[Q_from[i]] = i
    return false
end

function step(self::PIBT, Q_from::Config{<:Integer},Q_to::Config{<:Integer}, order::Vector{Int})::Bool
    flg_success = true
    for (index, (vᵢᶠʳᵒᵐ, vᵢᵗᵒ)) in enumerate(zip(Q_from, Q_to)) 
        self.occupied_now[vᵢᶠʳᵒᵐ] = index
        if vᵢᵗᵒ != self.NIL_COORD
            # check vertex collision
            if self.occupied_next[vᵢᵗᵒ] != self.NIL
                flg_success = false
                @debug "check vertex collision"
                break
            end
            j = self.occupied_now[vᵢᵗᵒ]
            if j != self.NIL && j != index && Q_to[j] == vᵢᶠʳᵒᵐ
                flg_success = false
                @debug "check edge collision"
                break
            end
            self.occupied_next[vᵢᵗᵒ] = index
        end
    end

    # perform PIBT
    if flg_success == true
        for i in order
            if Q_to[i] == self.NIL_COORD
                flg_success = funcPIBT(self, Q_from, Q_to, i)
                if flg_success == false
                    break
                end
            end
        end
    end

    # cleanup
    for (q_from, q_to) in zip(Q_from, Q_to)
        self.occupied_now[q_from] = self.NIL
        if q_to != self.NIL_COORD
            self.occupied_next[q_to] = self.NIL
        end
    end

    return flg_success
end

function run(self::PIBT, max_timestep::Integer = 1000) :: Configs
    # define priorities
    priorities = get.(self.dist_tables, self.starts) / length(self.grid)

    # main loop, generate sequence of configurations
    configs = [self.starts]
    while length(configs) <= max_timestep
        # obtain new configuration
        Q = step(self, configs[end], priorities)
        push!(configs, Q)

        # update priorities & goal check
        finishFlag = true
        for i in 1:self.N
            if Q[i] != self.goals[i]
                finishFlag = false
                priorities[i] += 1
            else
                priorities[i] -= floor(Real, priorities[i]) # round
            end
        end
        if finishFlag
            break  # goal
        end
    end

    return configs
end

# end # module