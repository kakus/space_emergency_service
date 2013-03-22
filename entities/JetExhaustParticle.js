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
      this.shape.shadow = new createjs.Shadow('#0096ff', 0, 0, 8);
   },

   startFading: function(time)
   {
      createjs.Tween.get(this.shape, {loop:false})
         .to({alpha: 1 }, 0, createjs.Ease.linear)
         .to({alpha: 0 }, time, createjs.Ease.linear);
   }

});
