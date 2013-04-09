/* global Ses */

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
      fixDef.filter.categoryBits = Ses.Physic.CATEGORY_WORLD;
      fixDef.filter.maskBits = Ses.Physic.WORLD_MASK;


      var v = [];
      var b = Ses.Physic.createDynamicBody(null, { x: x, y: y });

      while ( true )
      {
         try
         {
            if( size < 3 )
               v = this.createRandomConvexPolygon(size, size * 0.6, 16);
            else
               v = this.createRandomConvexPolygon(size, size * 0.75, 18);

      
            Box2D.Common.Separator.Separate(b, fixDef, v);
            break;
         }
         catch (err)
         {
            var x = Box2D.Common.Separator.Validate(v);
            Ses.log(Box2D.Common.Separator.validateToString(x)+' : '+size);
            //throw err;
         }
      }

      this.body = b;
      //fixDef.shape.SetAsArray(v);
      fixDef.filter.categoryBits = Ses.Physic.CATEGORY_WORLD;

      //this.body = Ses.Physic.createDynamicBody(
      //   fixDef,
      //   { x: x, y: y}
      //);

      this.initShape(v);
      this.body.SetUserData({
         hookAble: true
      });

      var pixelSize = size*Ses.Engine.Scale;
      switch (Ses.Engine.Graphics) {
      case 'low':
         this.shape.cache(-pixelSize, -pixelSize,
                           pixelSize*2, pixelSize*2);
         break;

      case 'medium':
         break;

      case 'high':
         break;
      }
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
      //graphics.beginFill('#444444');

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
         var lowerBound = n*range + range/3;
         var upperBound = (n+1)*range - range/3;
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
