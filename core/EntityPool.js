/* global Class */

Ses.Core.EntityPool = Class.extend({

   /**
    * @Entity class of entity that you want to have pool of. it must have a
    * default contructor that doesnt take any arguments
    * @poolNumber number of entities in your pool
    * @constructorArgs An object where each key is function that returns
    * argument value
    * eg. {
    *        a: function() { return 1; },
    *        b: ...          return Math.random(); }
    *     }
    */
   init: function(Entity, poolNumber, constructorArgs)
   {
      this.pool = [];


      for (var i = 0; i < poolNumber; ++i)
      {
         // a help function that makes constructor with given arguments
         // eg. new Wrap <=> new Entity.apply([0, 0, 3])
         var Wrap = (function() {
            var args = [];
            for (var arg in constructorArgs)
               args.push(constructorArgs[arg]());

            function F() {
               return Entity.apply(this, args);
            }
            F.prototype = Entity.prototype;
            return F;
         })();

         this.pool.push(new Wrap());
      }
   },

   getOne: function()
   {
      for (var i = 0; i < this.pool.length; ++i)
         if (this.pool[i].alive)
            return this.pool[i];
   }

});
