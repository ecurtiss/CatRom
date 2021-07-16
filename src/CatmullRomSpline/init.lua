-- TODO: Support inputting single numbers
-- TODO: Support inputting arbitrary tuples (n_0, n_1, ..., n_k)
-- TODO: Support variable caching of GetSplineFromAlpha depending on #splines in chain
-- TODO: Cache GetArcLengthAlpha
-- TODO: Add get nearest point on spline
-- TODO: Add get alpha from length
-- TODO: Fix name shadowing with alpha. alpha is a spline parameter but also percent along spline
-- FIX: Not good transitions between nodes on Arc methods

return {
	Spline = require(script.Spline),
	Chain = require(script.Chain)
}