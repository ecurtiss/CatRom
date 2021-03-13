local ReplicatedStorage = game:GetService("ReplicatedStorage")
local CatmullRomSpline = require(ReplicatedStorage.CatmullRomSpline)

local NUM_KNOTS = 10

local Knots = table.create(NUM_KNOTS)
local KnotsFolder = Instance.new("Folder")
KnotsFolder.Name = "Knots"
for i = 1, NUM_KNOTS do
	local part = Instance.new("Part")
	part.Anchored = true
	part.Size = Vector3.new(1, 1, 1)
	part.CFrame = CFrame.new(i * 2	, math.random() * 10, math.random() * 2) * CFrame.Angles(math.random(), math.random(), math.random())
	part.Shape = Enum.PartType.Ball
	part.Material = Enum.Material.Neon
	part.Parent = KnotsFolder
	Knots[i] = part.CFrame
end
KnotsFolder.Parent = workspace
local Chain = CatmullRomSpline.Chain.new(Knots)

local PartsFolder = Instance.new("Folder")
PartsFolder.Name = "SplineParts"
for i = 0, 100 do
	local alpha = i / 100
	local part = Instance.new("Part")
	part.Anchored = true
	part.Size = Vector3.new(1, 1, 1) * 0.5
	part.CFrame = Chain:SolveRotCFrame(alpha)
	part.Parent = PartsFolder
end
PartsFolder.Parent = workspace