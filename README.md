# jlLacam

このコードはKeisuke Okumura氏のpy-lacam-pibtをjuliaにベタ移植したものです。  
参照元は右記です。 [py-lacam-pibt](https://github.com/sasaya/lacam/tree/pibt)　Copyright 2024 (c) Keisuke Okumura


### How to use
- stand alone  
`julia --project=. app.jl -N 4`

- in code
    ```julia
    using jlLacam

    grid = fill(true, 2, 3)
    starts = [[1, 1], [1, 3]]
    goals = [[1, 3], [1, 1]]
    flg_star = true

    planner = LaCAMdata(grid, starts, goals, flg_star)
    solution = planner |> solve
    ```

