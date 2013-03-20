Ses.Entities.CircleSensor = Ses.Entities.Sensor.extend({

   init: function(x, y, radious)
   {
      this.body = Ses.Physic.createCircleSesnor(x, y, radious);

      var g = new createjs.Graphics();
      g.setStrokeStyle(1);//, 0, 0, 10, true);
      g.beginFill('rgba(255,255,0,0.1)');
      g.beginStroke('#ffff00');
      g.drawCircle(x,y,radious*Ses.Engine.Scale);
      this.shape = new createjs.Shape(g);

      createjs.Tween.get(this.shape, {loop:true})
         .to({scaleX: 0.95, scaleY: 0.95 }, 300, createjs.Ease.linear)
         .to({scaleX: 1, scaleY: 1 }, 300, createjs.Ease.linear);
      //this.shape.shadow = new createjs.Shadow('#ffff00', 0, 0, 4);
   }

});
