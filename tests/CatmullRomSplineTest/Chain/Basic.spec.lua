return function()

    local state = {}

    it("should create a new chain", function(context)
        local chain = context.CatmullRomSpline.Chain.new({
            Vector2.new(0, 0),
            Vector2.new(0, 1),
            Vector2.new(1, 1),
            Vector2.new(1, 0)
        })
        state.chain = chain

        expect(chain).to.be.ok()
    end)

    it("should get position", function(context)
        local position = state.chain:GetPosition(0.25)

        expect(position.X).to.be.near(0)
        expect(position.Y).to.be.near(0)
        expect(position.Z).to.be.near(0)
    end)

    it("should get cframe", function(context)
        local cframe = state.chain:GetCFrame(0.25)
        local x, y, z, m11, m12, m13, m21, m22, m23, m31, m32, m33 = cframe:components()

        expect(x).to.be.near(0)
        expect(y).to.be.near(0)
        expect(z).to.be.near(0)
        expect(m11).to.be.near(0)
        expect(m12).to.be.near(0)
        expect(m13).to.be.near(0)
        expect(m21).to.be.near(0)
        expect(m22).to.be.near(0)
        expect(m23).to.be.near(0)
        expect(m31).to.be.near(0)
        expect(m32).to.be.near(0)
        expect(m33).to.be.near(0)
    end)

    it("should get rotation", function(context)
        local cframe = state.chain:GetRotCFrame(0.25)
        local x, y, z, m11, m12, m13, m21, m22, m23, m31, m32, m33 = cframe:components()

        expect(x).to.be.near(0)
        expect(y).to.be.near(0)
        expect(z).to.be.near(0)
        expect(m11).to.be.near(0)
        expect(m12).to.be.near(0)
        expect(m13).to.be.near(0)
        expect(m21).to.be.near(0)
        expect(m22).to.be.near(0)
        expect(m23).to.be.near(0)
        expect(m31).to.be.near(0)
        expect(m32).to.be.near(0)
        expect(m33).to.be.near(0)
    end)

    it("should get curvature", function(context)
        local direction, magnitude = state.chain:GetCurvature(0.25)

        expect(direction.X).to.be.near(0)
        expect(direction.Y).to.be.near(0)
        expect(direction.Z).to.be.near(0)
        expect(magnitude).to.be.near(0)
    end)

    it("should get arc position", function(context)
        local position = state.chain:GetArcPosition(0.25)

        expect(position.X).to.be.near(0)
        expect(position.Y).to.be.near(0)
        expect(position.Z).to.be.near(0)
    end)

    it("should get arc cframe", function(context)
        local cframe = state.chain:GetArcCFrame(0.25)
        local x, y, z, m11, m12, m13, m21, m22, m23, m31, m32, m33 = cframe:components()

        expect(x).to.be.near(0)
        expect(y).to.be.near(0)
        expect(z).to.be.near(0)
        expect(m11).to.be.near(0)
        expect(m12).to.be.near(0)
        expect(m13).to.be.near(0)
        expect(m21).to.be.near(0)
        expect(m22).to.be.near(0)
        expect(m23).to.be.near(0)
        expect(m31).to.be.near(0)
        expect(m32).to.be.near(0)
        expect(m33).to.be.near(0)
    end)

    it("should get arc rotation", function(context)
        local cframe = state.chain:GetArcRotCFrame(0.25)
        local x, y, z, m11, m12, m13, m21, m22, m23, m31, m32, m33 = cframe:components()

        expect(x).to.be.near(0)
        expect(y).to.be.near(0)
        expect(z).to.be.near(0)
        expect(m11).to.be.near(0)
        expect(m12).to.be.near(0)
        expect(m13).to.be.near(0)
        expect(m21).to.be.near(0)
        expect(m22).to.be.near(0)
        expect(m23).to.be.near(0)
        expect(m31).to.be.near(0)
        expect(m32).to.be.near(0)
        expect(m33).to.be.near(0)
    end)

    it("should get arc curvature", function(context)
        local direction, magnitude = state.chain:GetArcCurvature(0.25)

        expect(direction.X).to.be.near(0)
        expect(direction.Y).to.be.near(0)
        expect(direction.Z).to.be.near(0)
        expect(magnitude).to.be.near(0)
    end)

end