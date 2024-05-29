local Utils = {}

--[=[
	Solves for the real roots of the quadratic polynomial at^2 + bt + c. Returns
	nil if there are no real roots. Returns only one real root if they are
	equal.
--]=]
function Utils.SolveQuadratic(a: number, b: number, c: number): (number?, number?)
	if a == 0 then
		if b == 0 then
			return nil
		else
			return -c / b
		end
	end

	local discriminant = b ^ 2 - 4 * a * c

	if discriminant == 0 then
		return -b / (2 * a), nil
	elseif discriminant > 0 then
		local negativeB = -b
		local sqrt = math.sqrt(discriminant)
		local doubleA = 2 * a

		return (negativeB - sqrt) / doubleA, (negativeB + sqrt) / doubleA
	end

	return nil, nil
end

return Utils