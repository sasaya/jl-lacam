# jlLacam

[![Build Status](https://github.com/sasaya/jlLacam.jl/actions/workflows/CI.yml/badge.svg?branch=master)](https://github.com/sasaya/jlLacam.jl/actions/workflows/CI.yml?query=branch%3Amaster)

このコードはKeisuke Okumura氏のpy-lacam-pibtをjuliaにベタ移植したものです。  
ライセンス元の[py-lacam-pibt](https://github.com/sasaya/lacam/tree/pibt)に準じます。

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

