---@diagnostic disable: undefined-global
--# selene: allow(undefined_variable)
--[[
	Copies the Spline methods into CatRom. If the following two lines are not
	present in the script,

	---- START GENERATED METHODS
	---- END GENERATED METHODS

	then the methods will not be written.
]]

local START_GEN = "---- START GENERATED METHODS"
local END_GEN = "---- END GENERATED METHODS"

local CATROM_FILE = "src/init.lua"
local SPLINE_FILE = "src/spline.lua"

-- Generate the methods
local Methods = {}

for line in io.lines(SPLINE_FILE) do
	-- Look for a method
	local start, stop = string.find(line, "function Spline:Solve(%a+)%(")
	if start then
		-- Get everything in the method's parentheses
		local inputs = string.match(string.sub(line, stop + 1), "(.+)%)")

		-- Get the input arguments
		local args = {}
		for arg in string.gmatch(inputs, "(%a+)(%:)") do
			table.insert(args, arg)
		end

		if #args == 1 and args[1] == "t" then
			local methodName = string.match(line, "(%a+)%(", 22)
			local method = {
				"function CatRom:Solve" .. methodName .. string.sub(line, stop),
				"\tlocal spline, splineT = self:GetSplineFromT(t)",
				"\treturn spline:Solve" .. methodName .. "(splineT)"
			}

			-- End the method
			table.insert(method, "end")

			table.insert(Methods, method)
		end
	end
end

-- Get the file data and replace the generated methods
local FileData = {}
local InGeneratedLines = false

for line in io.lines(CATROM_FILE) do
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
local file = io.open(CATROM_FILE, "w")

for i, line in ipairs(FileData) do
	if i == #FileData then
		-- Don't put a new line after the return statement
		file:write(line)
	else
		file:write(line .. "\n")
	end
end

file:close()