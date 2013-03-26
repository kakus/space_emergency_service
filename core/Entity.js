/*
 * Kamil Misiowiec 1.03.2013
 *
 * Entity - base class of every object that aprears in game.
 */

Ses.Core.Entity = Class.extend({
   init: function() {},
   update: function()
   {
      if(!this.body)
         return;

      this.shape.x = this.body.GetWorldCenter().x * Ses.Engine.Scale;
      this.shape.y = this.body.GetWorldCenter().y * Ses.Engine.Scale;
      this.shape.rotation = this.body.GetAngle() * (180/Math.PI);
   },

   draw: function(ctx)
   {
      throw new Error("This method should be overriden !");
   },

   getCustomDrawFunction: function(offsetX, offsetY)
   {
      var self = this;
      var offX = offsetX || 0;
      var offY = offsetY || 0;

      return function(ctx) {

         ctx.save();

            ctx.translate(offX, offY);
            self.draw(ctx);

         ctx.restore();
      };
   },

   body: null,
   shape: null
});
