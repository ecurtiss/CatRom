return function()

    local state = {}

    it("should create a new spline", function(context)
        local spline = context.CatmullRomSpline.Spline.new(
            Vector2.new(0, 0),
            Vector2.new(0, 1),
            Vector2.new(1, 1),
            Vector2.new(1, 0)
        )
        state.spline = spline

        expect(spline).to.be.ok()
    end)

    it("should solve for position", function(context)
        local position = state.spline:SolvePosition(0.25)

        expect(position.X).to.be.near(0)
        expect(position.Y).to.be.near(0)
        expect(position.Z).to.be.near(0)
    end)

    it("should solve for tangent", function(context)
        local tangent = state.spline:SolveTanget(0.25)

        expect(tangent.X).to.be.near(0)
        expect(tangent.Y).to.be.near(0)
        expect(tangent.Z).to.be.near(0)
    end)

    it("should solve for cframe", function(context)
        local cframe = state.spline:SolveCFrame(0.25)
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

    it("should solve for length", function(context)
        local length = state.spline:SolveLength(0.25)
        
        expect(length).to.be.near(0)
    end)

    it("should solve for curvature", function(context)
        local direction, magnitude = state.spline:SolveCurvature(0.25)

        expect(direction.X).to.be.near(0)
        expect(direction.Y).to.be.near(0)
        expect(direction.Z).to.be.near(0)
        expect(magnitude).to.be.near(0)
    end)

    it("should solve for rotation", function(context)
        local cframe = state.spline:SolveRotCFrame(0.25)
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

end