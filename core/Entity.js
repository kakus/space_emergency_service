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
   getSpeed: function() {
      if(!this.body)
         return this.body.GetLinearVelocity();
   },
   body: null,
   shape: null
});
