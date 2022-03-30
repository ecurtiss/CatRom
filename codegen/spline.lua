---@diagnostic disable: undefined-global
--# selene: allow(undefined_variable)
--[[
	Generates wrapper methods for the arc length parameterization of a CatRom.
	If the following two lines are not present in the script,

	---- START GENERATED METHODS
	---- END GENERATED METHODS

	then the methods will not be written.
]]

local START_GEN = "---- START GENERATED METHODS"
local END_GEN = "---- END GENERATED METHODS"

local SPLINE_FILE = "src/Spline.lua"

-- Generate the methods
local Methods = {}

local InGeneratedLines = false
for line in io.lines(SPLINE_FILE) do
	if line == START_GEN then -- Ignore the methods already generated
		InGeneratedLines = true
	elseif line == END_GEN then
		InGeneratedLines = false
	elseif not InGeneratedLines then
		-- Look for a method
		local start, stop = string.find(line, "function Spline:Solve(%a+)%(")
		if start then
			local methodName = string.match(line, "(%a+)%(", 22)
			local method = {
				"function Spline:SolveUniform" .. methodName .. string.sub(line, stop)
			}

			-- Get everything in the method's parentheses
			local inputs = string.match(string.sub(line, stop + 1), "(.+)%)")

			-- Call the method
			local methodCall = string.format("\treturn self:Solve%s(", methodName)

			-- Reparameterize the arguments
			local i = 1
			for arg in string.gmatch(inputs, "(%a+)(%:)") do
				methodCall = methodCall .. (i > 1 and ", " or "") .. string.format("self:Reparameterize(%s)", arg)
				i = i + 1
			end

			-- Finish the method call
			methodCall = methodCall .. ")"
			table.insert(method, methodCall)

			-- End the method
			table.insert(method, "end")

			table.insert(Methods, method)
		end
	end
end

-- Get the file data and replace the generated methods
local FileData = {}
InGeneratedLines = false

for line in io.lines(SPLINE_FILE) do
	if line == START_GEN then
		table.insert(FileData, line)
		-- Write the methods
		for _, method in ipairs(Methods) do
			for _, methodLine in ipairs(method) do
				table.insert(FileData, methodLine)
			end
		end
		InGeneratedLines = true
	elseif line == END_GEN then
		table.insert(FileData, line)
		InGeneratedLines = false
	elseif not InGeneratedLines then
		table.insert(FileData, line)
	end
end

-- Write to the file
local file = io.open(SPLINE_FILE, "w")

for i, line in ipairs(FileData) do
	if i == #FileData then
		-- Don't put a new line after the return statement
		file:write(line)
	else
		file:write(line .. "\n")
	end
end

file:close()