-- Legendre roots and quadrature weights sourced from
-- https://pomax.github.io/bezierinfo/legendre-gauss.html
-- with highest precision afforded by 64-bit floats.

local GaussLegendre = {}

function GaussLegendre.Five(f, a, b)
	if a == b then
		return 0
	end

	local min = math.min(a, b)
	local max = math.max(a, b)

	local halfInterval = (max - min) / 2
	local midpoint = (max + min) / 2

	local x2 = halfInterval * 0.5384693101056831
	local x3 = halfInterval * 0.906179845938664

	return halfInterval * (
		  0.5688888888888889  * f(midpoint)
		+ 0.47862867049936647 * (f(midpoint - x2) + f(midpoint + x2))
		+ 0.23692688505618908 * (f(midpoint - x3) + f(midpoint + x3))
	)
end

function GaussLegendre.Ten(f, a, b)
	if a == b then
		return 0
	end

	local min = math.min(a, b)
	local max = math.max(a, b)

	local halfInterval = (max - min) / 2
	local midpoint = (max + min) / 2

	local x1 = halfInterval * 0.14887433898163122
	local x2 = halfInterval * 0.4333953941292472
	local x3 = halfInterval * 0.6794095682990244
	local x4 = halfInterval * 0.8650633666889845
	local x5 = halfInterval * 0.9739065285171717

	return halfInterval * (
		  0.29552422471475287 * (f(midpoint - x1) + f(midpoint + x1))
		+ 0.26926671930999635 * (f(midpoint - x2) + f(midpoint + x2))
		+ 0.21908636251598204 * (f(midpoint - x3) + f(midpoint + x3))
		+ 0.1494513491505806  * (f(midpoint - x4) + f(midpoint + x4))
		+ 0.06667134430868814 * (f(midpoint - x5) + f(midpoint + x5))
	)
end

function GaussLegendre.Twenty(f, a, b)
	if a == b then
		return 0
	end

	local min = math.min(a, b)
	local max = math.max(a, b)

	local halfInterval = (max - min) / 2
	local midpoint = (max + min) / 2

	local x1 = halfInterval * 0.07652652113349734
	local x2 = halfInterval * 0.22778585114164507
	local x3 = halfInterval * 0.37370608871541955
	local x4 = halfInterval * 0.5108670019508271
	local x5 = halfInterval * 0.636053680726515
	local x6 = halfInterval * 0.746331906460150
	local x7 = halfInterval * 0.839116971822218
	local x8 = halfInterval * 0.912234428251326
	local x9 = halfInterval * 0.963971927277913
	local x10 = halfInterval * 0.9931285991850949

	return halfInterval * (
		  0.15275338713072584  * (f(midpoint - x1)  + f(midpoint + x1))
		+ 0.14917298647260374  * (f(midpoint - x2)  + f(midpoint + x2))
		+ 0.14209610931838204  * (f(midpoint - x3)  + f(midpoint + x3))
		+ 0.13168863844917664  * (f(midpoint - x4)  + f(midpoint + x4))
		+ 0.11819453196151841  * (f(midpoint - x5)  + f(midpoint + x5))
		+ 0.10193011981724044  * (f(midpoint - x6)  + f(midpoint + x6))
		+ 0.08327674157670475  * (f(midpoint - x7)  + f(midpoint + x7))
		+ 0.06267204833410907  * (f(midpoint - x8)  + f(midpoint + x8))
		+ 0.0406014298003869   * (f(midpoint - x9)  + f(midpoint + x9))
		+ 0.017614007139152118 * (f(midpoint - x10) + f(midpoint + x10))
	)
end

return GaussLegendre