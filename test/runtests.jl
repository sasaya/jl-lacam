using jlLacam
using Test
using BenchmarkTools

@testset "test_get_grid" begin
    map_name = joinpath(@__DIR__, "assets", "3x2.map")
    grid = get_grid(map_name)
    @test size(grid) == (2, 3)
    @test grid == [false true true; true true true]
end

@testset "test_get_scenario" begin
    scen_name = joinpath(@__DIR__, "assets", "3x2.scen") 
    starts, goals = get_scenario(scen_name)
    @test length(starts) == 2
    @test length(goals) == 2
    # (y, x)
    @test starts[1] == [1, 2] && goals[1] == [2, 3]
    @test starts[2] == [2, 3] && goals[2] == [2, 1]

    starts, goals = get_scenario(scen_name, N=1)
    @test length(starts) == 1

    starts, goals = get_scenario(scen_name, N=3)
    @test length(starts) == 2
end

@testset "test_isvalid_coordinate" begin
    map_name = joinpath(@__DIR__, "assets", "3x2.map")
    grid = get_grid(map_name)
    @test isvalid_coordinate(grid, [2, 2])
    @test !isvalid_coordinate(grid, [1, 1])
    @test !isvalid_coordinate(grid, [0, 1])
    @test !isvalid_coordinate(grid, [3, 1])
    @test !isvalid_coordinate(grid, [1, 0])
    @test !isvalid_coordinate(grid, [1, 4])
end

@testset "test_get_neighbors" begin
    map_name = joinpath(@__DIR__, "assets", "3x2.map")
    grid = get_grid(map_name)
    # (y, x)
    neigh = get_neighbors(grid, [1, 2])
    @test length(neigh) == 2
    @test [2, 2] in neigh && [1, 3] in neigh

    neigh = get_neighbors(grid, [2, 2])
    @test length(neigh) == 3
    @test [1, 2] in neigh && [2, 1] in neigh && [2, 3] in neigh

    # invalid check
    neigh = get_neighbors(grid, [0, 0])
    println(neigh)
    @test length(neigh) == 0

    neigh = get_neighbors(grid, [3, 1])
    @test length(neigh) == 0

    neigh = get_neighbors(grid, [2, 4])
    @test length(neigh) == 0
end

@testset "test_save_configs_for_visualizer" begin
    configs = [
        [[0, 1], [1, 2]],
        [[0, 2], [1, 1]],
        [[1, 2], [1, 0]],
    ]
    # 実装されていない機能の場合、ダミー実装
    # TODO: 真の実装を追加
end

@testset "test_is_valid_mapf_solution" begin
    map_name = joinpath(@__DIR__, "assets", "3x2.map")
    scen_name = joinpath(@__DIR__, "assets", "3x2.scen") 
    grid = get_grid(map_name)
    starts, goals = get_scenario(scen_name)

    # Feasible solution
    configs = [
        [[1, 2], [2, 3]],
        [[1, 3], [2, 2]],
        [[2, 3], [2, 1]],
    ]
    @test is_valid_mapf_solution(grid, starts, goals, configs)

    # Invalid starts
    configs = [
        [[1, 3], [2, 3]],
        [[1, 3], [2, 2]],
        [[2, 3], [2, 1]],
    ]
    @test !is_valid_mapf_solution(grid, starts, goals, configs)

    # Invalid goals
    configs = [
        [[1, 2], [2, 3]],
        [[1, 3], [2, 2]],
        [[2, 3], [2, 2]],
    ]
    @test !is_valid_mapf_solution(grid, starts, goals, configs)

    # Non-connected path
    configs = [
        [[1, 2], [2, 3]],
        [[2, 3], [2, 2]],
        [[2, 3], [2, 1]],
    ]
    @test !is_valid_mapf_solution(grid, starts, goals, configs)

    # Vertex collision
    configs = [
        [[1, 2], [2, 3]],
        [[2, 2], [2, 2]],
        [[2, 3], [2, 1]],
    ]
    @test !is_valid_mapf_solution(grid, starts, goals, configs)

    # Edge collision
    configs = [
        [[1, 2], [2, 3]],
        [[2, 2], [2, 3]],
        [[2, 3], [2, 2]],
        [[2, 3], [2, 1]],
    ]
    @test !is_valid_mapf_solution(grid, starts, goals, configs)
end


@testset "DistTable" begin
    map_name = joinpath(@__DIR__, "assets", "3x2.map")
    grid = get_grid(map_name)
    goal = [2, 3]

    dist_table = DistTable(grid, goal)
    @test get(dist_table,goal) == 0
    @test get(dist_table,[2, 1]) == 2
    @test get(dist_table,[1, 1]) == 6  # invalid coordination
    @test get(dist_table,[1, 4]) == 6  # invalid coordination
end

@testset "LaCAM solve" begin
    grid = fill(true, 2, 3)
    starts = [[1, 1], [1, 3]]
    goals = [[1, 3], [1, 1]]
    flg_star = true
    planner = LaCAMdata(grid, starts, goals, flg_star)
    solution = planner |> solve
    @test is_valid_mapf_solution(grid, starts, goals, solution) == true
    @test get_sum_of_loss(solution) == 6
end
