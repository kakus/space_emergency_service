/*
 * Kamil Misiowiec 1.03.2013
 *
 * Entity - base class of every object that aprears in game.
 *
 */

/* global Class */

Ses.Core.Entity = Class.extend({

   maxHitPoints: 100,
   currentHitPoints: 10,
   alive: true,

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

   hit: function(damage)
   {
      this.currentHitPoints -= damage;
      if (this.currentHitPoints <= 0)
         this.setDead();
   },

   getLifeInPrecetage: function()
   {
      if (this.currentHitPoints < 1)
         return 0;

      return this.currentHitPoints/this.maxHitPoints;
   },

   setDead: function()
   {
      if (!this.alive)
         return;

      this.alive = false;
      if (this.onDieCallback)
         this.onDieCallback.call();
   },

   setOnDieListener: function(callback)
   {
      this.onDieCallback = callback;
   },

   /**
    * @property which we are listening for a change
    * @callback a function that will be called when property change
    */
   watch: function(property, callback)
   {
      // if this object doesnt have the property just return
      if (!this[property])
         return;

      if (this[property+'_listeners'])
         this[property+'_listeners'].push(callback);
      else
      {
         var listeners = this[property+'_listeners'] = [callback];
         var prop = this[property+'_'] = this[property];
      }

      var addSetterToPrototype = function(property) {
         // if the object isnt owner of property, we must go deeper in chain of
         // inheritance
         if (!this.hasOwnProperty(property))
            addSetterToPrototype.call(this.__proto__, property);

         this.__defineSetter__(property, function(val) {
            for(var i = 0; i < listeners.length; ++i)
               listeners[i].call({}, prop, val);
            prop = val;
         });

         this.__defineGetter__(property, function() {
            return prop;
         });
      };

      addSetterToPrototype.call(this, property);
   }
});
