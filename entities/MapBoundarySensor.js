Ses.Engine.MapBoundarySensor = Ses.Entities.Sensor.extend({

   init: function(width, height)
   {
      if ( (typeof width !== 'number') || (typeof height !== 'number') )
         throw new Error('Width or Height is not number');

      this.body = Ses.Physic.createRectangleSesnor(width, height, width, height);

      var g = new createjs.Graphics();
      g.setStrokeStyle(1, 0, 0, 10, true);
      g.beginStroke('#b03c3c');
      var w = width * Ses.Engine.Scale;
      var h = height * Ses.Engine.Scale;
      g.rect(-w, -h, w*2, h*2);
      this.shape = new createjs.Shape(g);
      this.shape.shadow = new createjs.Shadow('#b03c3c', 0, 0, 4);
   },

});
