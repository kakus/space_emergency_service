Ses.Entities.SpaceRock = Ses.Core.Entity.extend({

   init: function (x, y, size)
   {
      this._super();

      // physic
      var fixDef = new Ses.b2FixtureDef();
      fixDef.density = 1;
      fixDef.friction = 0.5;
      fixDef.friction = 0.2;
      fixDef.shape = new Ses.b2PolygonShape();

      var v = this.createRandomConvexPolygon(size, size * 0.9, 8);
      fixDef.shape.SetAsArray(v);
      fixDef.filter.categoryBits = Ses.Physic.CATEGORY_WORLD;

      this.body = Ses.Physic.createDynamicBody(
         fixDef,
         { x: x, y: y}
      );

      this.initShape(v);
      this.body.SetUserData('rock');
   },

   initShape: function(vertexs)
   {
      var massCenter = this.body.GetLocalCenter();
      var transform = function(v) {
         return { x: (v.x - massCenter.x) * Ses.Engine.Scale,
                  y: (v.y - massCenter.y) * Ses.Engine.Scale };
      };

      var graphics = new createjs.Graphics();
      graphics.beginStroke('#FFFFFF');

      var p = transform(vertexs[0]);
      graphics.moveTo(p.x, p.y);
      for(var i=1; i < vertexs.length; ++i)
      {
         p = transform(vertexs[i]);
         graphics.lineTo(p.x, p.y);
      }
      graphics.closePath();
      graphics.endStroke();

      this.shape = new createjs.Shape(graphics);
   },

   update: function()
   {
      this._super();
   },

   createRandomConvexPolygon:
      function(externalRadious, internalRadious, numberOfVertex)
   {
      var distance = function(x, y) {
         return Math.sqrt(x*x + y*y);
      };

      // in radians
      var range = 2*Math.PI / numberOfVertex;
      var vertexs = [];

      for(var n=0; n < numberOfVertex; ++n)
      {
         var lowerBound = n*range;
         var upperBound = (n+1)*range;
         var x, y;

         while(true)
         {
            x = Math.random()*externalRadious;
            x *= Math.random() > 0.5 ? 1 : -1;
            y = Math.random()*externalRadious;
            y *= Math.random() > 0.5 ? 1 : -1;
            var d = distance(x, y);

            if(d < internalRadious || d > externalRadious)
               continue;

            var r = Math.atan2(y, x);
            if(lowerBound < Math.PI)
            {
               if(r > lowerBound && r < upperBound)
                  break;
            }
            else
            {
               var lb = -Math.PI + (lowerBound-Math.PI);
               var up = lb + range;
               if(r > lb && r < up)
                  break;
            }
         }

         vertexs.push(new Ses.b2Vec2(x, y));
      }

      return vertexs;
   }
});
