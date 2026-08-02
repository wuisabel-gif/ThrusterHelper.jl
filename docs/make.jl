using Documenter
using ThrusterHelper

makedocs(;
    sitename = "ThrusterHelper.jl",
    modules = [ThrusterHelper],
    authors = "Isabel Wu",
    format = Documenter.HTML(;
        canonical = "https://wuisabel-gif.github.io/ThrusterHelper.jl",
        prettyurls = get(ENV, "CI", nothing) == "true",
    ),
    pages = [
        "Home" => "index.md",
        "API reference" => "api.md",
    ],
    checkdocs = :none,   # docstrings live all over src/; don't fail on any gaps
    warnonly = true,     # keep the first-pass build robust to stray @refs
)

deploydocs(;
    repo = "github.com/wuisabel-gif/ThrusterHelper.jl",
    devbranch = "main",
)
