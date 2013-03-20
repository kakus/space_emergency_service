Ses.Entities.RectangleSensor = Ses.Entities.Sensor.extend({

   init: function(x, y, width, height)
   {
      this.body = Ses.Physic.createRectangleSesnor(x, y, width, height);

      var g = new createjs.Graphics();
      //g.setStrokeStyle(1);//, 0, 0, 10, true);
      g.beginFill('rgba(0,255,0,0.1)');
      g.beginStroke('rgba(0,255,0,0.51)');
      //g.drawCircle(x,y,radious*Ses.Engine.Scale);
      g.rect(-width*30,-height*30,width*30*2, height*30*2);
      this.shape = new createjs.Shape(g);

      //createjs.Tween.get(this.shape, {loop:true})
      //   .to({scaleX: 0.95, scaleY: 0.95 }, 300, createjs.Ease.linear)
      //   .to({scaleX: 1, scaleY: 1 }, 300, createjs.Ease.linear);
      this.shape.shadow = new createjs.Shadow('#00ff00', 0, 0, 4);
   }

});
