local ReplicatedStorage = game:GetService("ReplicatedStorage")
local CatmullRomSpline = require(ReplicatedStorage.CatmullRomSpline)

local Parts = workspace.Folder:GetChildren()
local Knots = table.create(#Parts)
for i, part in ipairs(Parts) do
	Knots[i] = part.CFrame
end
local Chain = CatmullRomSpline.Chain.new(Knots)
print(Chain)

local Folder = Instance.new("Folder")
Folder.Name = "SplineParts"
for i = 0, 100 do
	local alpha = i / 100
	local position = Chain:GetPosition(alpha)

	local part = Instance.new("Part")
	part.Anchored = true
	part.Size = Vector3.new(1, 1, 1) * 0.5
	part.CFrame = Chain:GetArcRotCFrame(alpha)
	part.Parent = Folder
end
Folder.Parent = workspace