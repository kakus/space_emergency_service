Ses.Entities.CircleSensor = Ses.Entities.Sensor.extend({

   init: function(x, y, radious)
   {
      this.radious = radious;

      this.body = Ses.Physic.createCircleSesnor(x, y, radious,
         {
            filter: {
               maskBits: Ses.Physic.SENSOR_MASK,
               categoryBits: Ses.Physic.CATEGORY_SENSOR
            }
         }
      );

      var g = new createjs.Graphics();
      g.setStrokeStyle(1);//, 0, 0, 10, true);
      //g.beginFill('rgba(255,255,0,0.1)');
      g.beginStroke('#ffff00');
      //g.beginStroke('#ffffff');
      g.drawCircle(0,0,radious*Ses.Engine.Scale);
      this.shape = new createjs.Shape(g);
      this.shape.x = x;
      this.shape.y = y;

      createjs.Tween.get(this.shape, {loop:true})
         .to({scaleX: 0.95, scaleY: 0.95 }, 300, createjs.Ease.linear)
         .to({scaleX: 1, scaleY: 1 }, 300, createjs.Ease.linear);

      var pixelRadious = radious*Ses.Engine.Scale;

      switch (Ses.Engine.Graphics) {
      //case 'low':
      //   this.shape.cache(-pixelRadious, -pixelRadious,
      //                    pixelRadious*2, pixelRadious*2);
      //   break;

      case 'high':
         this.shape.shadow = new createjs.Shadow('#ffff00', 0, 0, 8);
         break;
      }
   },

   fadeOut: function(onCompleteCallback)
   {
      this.alive = false;
      createjs.Tween.removeTweens(this.shape);
      createjs.Tween.get(this.shape, {loop:false})
         .to({alpha: 0, scaleX: 2, scaleY: 2}, 300, createjs.Ease.linear)
         .call( onCompleteCallback );

   },

   notCheckpoint: function()
   {
      this.shape.graphics.clear()
         .s('#ffa800').dc(0, 0, this.radious*Ses.Engine.Scale);
      var self = this;
      this.setOnObjectEnterListener(function() {
         self.fadeOut(function() {
            self.shape.getStage().removeGameObject(self);
         });
      });
   }

});
