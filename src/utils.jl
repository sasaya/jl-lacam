function get_grid(map_file::String) ::Matrix{Bool}
    width, height = 0, 0
    
    f = open(map_file)
    for row in eachline(f)
        match_width = match(r"width\s(\d+)", row)
        if match_width !== nothing
            width = parse(Int, match_width[1])
        end
        
        match_height = match(r"height\s(\d+)", row)
        if match_height !== nothing
            height = parse(Int, match_height[1])
        end
        
        if width > 0 && height > 0
            break
        end
    end
    grid = Matrix{Bool}(undef, (height, width))
    y = 1
    seekstart(f)
    for row in eachline(f)
        row = strip(row)
        if length(row) == width && row != "map"
            grid[y,:] = [s == '.' for s in row]
            y += 1
        end
    end

    @assert y > height "Map format seems strange, check $map_file"
    close(f)

    return grid
end

function get_scenario(scen_file::String; N::Union{Nothing, Int}=nothing)
    starts  = Config{Int64}(undef,0)
    goals   = Config{Int64}(undef,0)
    open(scen_file, "r") do f
        for row in eachline(f)
            match_line = match(r"\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+", row)
            if match_line !== nothing
                x_s, y_s, x_g, y_g = parse.(Int, [match_line[1], match_line[2], match_line[3], match_line[4]])
                push!(starts, [y_s + 1, x_s + 1])
                push!(goals, [y_g + 1, x_g + 1])
                
                if N !== nothing && length(starts) >= N
                    break
                end
            end
        end
    end
    return starts, goals
end

function save_configs_for_visualizer(configs::Configs, filename::String) :: Nothing
    directoryname = dirname(filename)
    if !isdir(directoryname) && !isempty(directoryname)
        mkdir(directoryname)
    end
    f = open(filename, "w+")
        for (t, config) in enumerate(configs)
            row = "$t:" * join([ "($(y - 1), $(x - 1))," for (y, x) in config ], "") * "\n"
            write(f, row)
        end
    close(f)
end

function validate_mapf_solution(grid::Matrix{Bool}, starts::Config, goals::Config, solution::Configs)

    @assert all(u == v for (u, v) in zip(starts, solution[begin])) "Invalid solution, check starts"
    @assert all(u == v for (u, v) in zip(goals, solution[end])) "Invalid solution, check goals"

    T = length(solution)
    N = length(starts)
    
    for t in 1:T
        for i in 1:N
            v_i_now = solution[t][i]
            v_i_pre = solution[max(t - 1, 1)][i]
            
            neighbors = get_neighbors(grid, v_i_pre)

            @assert v_i_now in vcat([v_i_pre], neighbors) "Invalid solution, check connectivity"

            for j in (i+1):N
                v_j_now = solution[t][j]
                v_j_pre = solution[max(t - 1, 1)][j]
                @assert v_i_now != v_j_now "Invalid solution, check vertex collision"
                @assert !(v_i_now == v_j_pre && v_i_pre == v_j_now) "invalid solution, edge collision"
            end
        end
    end
    
    return nothing
end

function is_valid_mapf_solution(grid::Matrix{Bool}, starts::Config, goals::Config, solution::Configs)::Bool
    try
        validate_mapf_solution(grid, starts, goals, solution)
        return true
    catch e
        println(e)
        return false
    end
end

function get_sum_of_loss(configs:: Configs)::Int
    cost = 0
    for t in range(2, length(configs))
        @debug ([v_from for (v_from, v_to, goal) in zip(configs[t - 1], configs[t], configs[end])])
        cost += sum(.![(v_from == v_to == goal) for (v_from, v_to, goal) in zip(configs[t - 1], configs[t], configs[end])])
    end
    return cost
end

function get_neighbors(grid::Matrix{Bool}, coordinate::Vector{Int}) ::Vector{Vector{Int}}
    neigh = Vector{Vector{Int}}(undef,0)

    if !isvalid_coordinate(grid, coordinate)
        return neigh
    end

    y, x = coordinate

    if (x > 1 && grid[y, x - 1]) 
        push!(neigh, [y, x - 1]) 
    end
    
    if (x < size(grid,2) && grid[y, x + 1])
        push!(neigh, [y, x + 1])
    end

    if (y > 1 && grid[y - 1, x])
        push!(neigh, [y - 1, x])
    end

    if (y < size(grid,1) && grid[y + 1, x])
        push!(neigh, [y + 1, x])
    end

    return neigh
end

function isvalid_coordinate(grid::Matrix{Bool}, coordinate::Vector{Int}) ::Bool
    y,x = coordinate
    if (1 > y || 
        y > size(grid,1) ||
        1 > x || 
        x > size(grid,2) || 
        !grid[y,x]
    )
        return false
    end
    return true
end
# end # module