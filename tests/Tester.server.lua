--// CONSTANTS //--

local REPLICATED_STORAGE = game:GetService("ReplicatedStorage")
local TestEZ = require(REPLICATED_STORAGE.TestEZ)

--// VARIABLES //--



--// FUNCTIONS //--



--// INSTRUCTIONS //--

-- TestEZ.TestBootstrap:run({
--     script.Parent:FindFirstChild("Unorganized.spec")
-- })

TestEZ.TestBootstrap:run({
    script.Parent:FindFirstChild("CatmullRomSplineTest")
})