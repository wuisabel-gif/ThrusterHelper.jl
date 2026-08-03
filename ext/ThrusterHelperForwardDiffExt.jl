module ThrusterHelperForwardDiffExt

# Package extension: loaded automatically when both ThrusterHelper and ForwardDiff
# are present. Provides the exact automatic-differentiation gradient for
# `sensitivity(...; method=:ad)` (manipulability, directions only).

using ThrusterHelper
using ThrusterHelper: _encode, _manip_from_params, Thruster
import ForwardDiff

# More specific than the core (Any, Any) fallback, so the two coexist without
# an illegal precompile-time method overwrite.
function ThrusterHelper._ad_manip_grad(thr::AbstractVector{<:Thruster}, free::Symbol)
    free === :directions ||
        throw(ArgumentError("method=:ad supports free=:directions only; " *
                            "use method=:finitediff for positions"))
    angles = _encode(thr, :directions)
    positions = [t.position for t in thr]
    return ForwardDiff.gradient(a -> _manip_from_params(a, positions), angles)
end

end # module
