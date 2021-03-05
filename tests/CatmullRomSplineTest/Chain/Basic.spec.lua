return function()

    local state = {}

    it("should create a new chain", function(context)
        local chain = context.CatmullRomSpline.Spline.new(
            Vector2.new(0, 0),
            Vector2.new(0, 1),
            Vector2.new(1, 1),
            Vector2.new(1, 0)
        )
        state.chain = chain

        expect(chain).to.be.ok()
    end)

end