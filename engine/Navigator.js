/* global Class */

Ses.Engine.Navigator = Class.extend({

   init: function(fakeStage)
   {
      this.fakeStage = fakeStage;
      this.layer  = new createjs.Container();
      fakeStage.getStage().addChild(this.layer);
      this.trackedEntities = [];
      this.counter = 0;

      // ugly but we must call this to resolve the size of navigator
      this.onWindowResize();
   },

   setCenter: function(center)
   {
      this.center = center;
   },

   update: function()
   {
      if (!this.center) return;

      // we dont need to update this every frame
      if (++this.counter < 3)
         return;
      else
         this.counter = 0;

      for(var i = 0; i < this.trackedEntities.length; ++i)
      {
         var mark = this.trackedEntities[i];
         var dist = mark.position.Copy();
         dist.Subtract(this.center);
         var pixelDist = dist.Length()*Ses.Engine.Scale;
         var r = this.radious*1/this.fakeStage.scaleX;

         if (r > pixelDist - 100)
         {
            mark.shape.visible = false;
            continue;
         }
         else
            mark.shape.visible = true;

         mark.text.text = dist.Length().toFixed(1) + 'm';
         var p = this.getMarkPosition(dist);

         var c =
         {
            x: Ses.Engine.ScreenWidth/2,
            y: Ses.Engine.ScreenHeight/2,
         };

         mark.arrow.rotation = Math.atan2(p.y, p.x) * 180/Math.PI + 90;
         mark.arrow.x = p.x * 30;
         mark.arrow.y = p.y * 30;
         p.Multiply( this.radious );
         p.Add(c);
         mark.shape.x = p.x;
         mark.shape.y = p.y;
      }
   },

   track: function(entity)
   {
      var e =
      {
         color:      entity.trackColor || 'rgba(0, 158, 255, 0.5)',//#009eff',
         position:   entity.body.GetWorldCenter(),
         shape:      new createjs.Container()
      };

      var text = new createjs.Text('dst', '14px TitilliumText25L400wt', e.color);
      //text.shadow = new createjs.Shadow('#ffff00', 0, 0, 1);
      text.x = -15;
      text.y = -5;
      e.text = text;
      e.shape.addChild(text);

      var arrow = new createjs.Shape();
      arrow.graphics
         .f(e.color)
         .mt(  0,   0)
         .lt( -5,   5)
         .lt(  0, -10)
         .lt(  5,   5)
         .lt(  0,   0)
         .closePath();
      e.shape.addChild(arrow);
      e.arrow = arrow;

      this.layer.addChild(e.shape);
      this.trackedEntities.push(e);
   },

   /**
    * Calculates position on screen for a mark that shows direction and distance
    * to the target
    *
    * @vector is the vecotr between center of circle/screen and tracked object
    */
   getMarkPosition: function(vector)
   {
      var a = 1 / vector.Length();
      var p = vector.Copy();
      p.Multiply(a);
      return p;
   },

   hide: function()
   {
      this.layer.visible = false;
   },

   onWindowResize: function()
   {
      var offset = 150;
      var r = Math.min(Ses.Engine.ScreenHeight, Ses.Engine.ScreenWidth);
      this.radious = (r-offset)/2;
   }

});
