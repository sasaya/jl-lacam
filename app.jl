using ArgParse

using jlLacam

s = ArgParseSettings()

@add_arg_table! s begin
    "--map-file", "-m"
        help = "Map file path"
        default = joinpath(dirname(@__FILE__), "src", "assets", "tunnel.map")
    "--scen-file", "-i"
        help = "Scenario file path" 
        default = joinpath(dirname(@__FILE__), "src", "assets", "tunnel.scen")
    "--num-agents", "-N"
        help = "Number of agents"
        arg_type = Int
        default = 2
    "--output-file", "-o"
        help = "Output file name"
        default = "output.txt"
    "--verbose", "-v"
        help = "Verbosity level"
        arg_type = Int
        default = 1
    "--seed", "-s"
        help = "Random seed"
        arg_type = Int
        default = 0
    "--time_limit_ms", "-t"
        help = "Time limit in milliseconds"
        arg_type = Int
        default = 3000
    "--flg_star"
        help = "Use LaCAM* (default) or vanilla LaCAM"
        arg_type = Bool
        default = true
end

parsed_args = parse_args(s)

# Get grid and scenario
grid = get_grid(parsed_args["map-file"])
starts, goals = get_scenario(parsed_args["scen-file"], N=parsed_args["num-agents"])

# Solve MAPF problem
planner = LaCAMdata(
    grid=grid,
    starts=starts,
    goals=goals,
    seed=parsed_args["seed"],
    time_limit_ms=parsed_args["time_limit_ms"],
    flg_star=parsed_args["flg_star"],
    verbose=parsed_args["verbose"]
)
solution = planner |> solve

# Validate and save results
validate_mapf_solution(grid, starts, goals, solution)
save_configs_for_visualizer(solution, parsed_args["output-file"])
