Ses.Entities.JetExhaustParticle = Ses.Core.Entity.extend({

   init: function(x, y, radious)
   {
      this.body = Ses.Physic.createCircleObject(

         radious,
         {x: x, y: y},
         {
            density: 0.1,
            filter: {
               maskBits: Ses.Physic.JET_PARTICLE_MASK,
               categoryBits: Ses.Physic.CATEGORY_JET_PARTICLE,
            }
         }
      );
      this.body.SetBullet(true);

      var g = new createjs.Graphics();
      g.beginStroke('#0096ff');
      g.setStrokeStyle(1);
      g.drawCircle(0, 0, radious*Ses.Engine.Scale);
      this.shape = new createjs.Shape(g);

      switch (Ses.Engine.Graphics) {
      case 'high':
         this.shape.shadow = new createjs.Shadow('#0096ff', 0, 0, 8);
         break;
      }
      //this.shape.cache(-radious*30-5, -radious*30-5, radious*60+10, radious*60+10, 2);
   },

   startFading: function(time)
   {
      var self = this;
      self.alive = false;
      createjs.Tween.get(this.shape, {loop:false})
         .to({alpha: 1 }, 0, createjs.Ease.linear)
         .to({alpha: 0 }, time, createjs.Ease.linear)
         .call(function() {
            self.alive = true;
            self.body.SetActive(false);
            self.shape.visible = false;
         });
   }

});
