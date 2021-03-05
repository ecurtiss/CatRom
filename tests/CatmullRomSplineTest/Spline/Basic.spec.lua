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

end