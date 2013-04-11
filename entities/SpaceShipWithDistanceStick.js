Ses.Entities.SpaceShipWithDistanceStick = Ses.Core.Entity.extend({

   init: function(ship)
   {
      this.ship = ship;
      this.shape = new createjs.Container();
      this.body = this.ship.body;
      this.graspers = [];

      var backgorundShape = new createjs.Shape();
      this.background = backgorundShape.graphics;

      this.shape.addChild(backgorundShape);
      this.shape.addChild(ship.shape);
   },

   update: function(stage)
   {
      this.ship.update(stage);

      if (this.graspers.length > 0)
      {
         this.background.clear();
         for (var i = 0; i < this.graspers.length; ++i)
         {
            var gr = this.graspers[i];

            if (gr.outOfRange)
            {
               this.graspers.splice(i, 1);
               stage.removeGameObject(gr);
               continue;
            }

            var gp = gr.body.GetWorldCenter();
            var sp = this.ship.body.GetWorldCenter();

            this.background
               .setStrokeStyle(1)
               .beginStroke('#0096ff')
               .moveTo(sp.x*30, sp.y*30)
               .lineTo(gp.x*30, gp.y*30);

            gr.update(stage);
         }
      }

      if (stage.RemoveGraspers)
      {
         for (var j = 0; j < this.graspers.length; ++j)
         {
            this.graspers[j].detach();
            stage.removeGameObject(this.graspers[j]);
         }
         this.graspers = [];
         this.background.clear();
      }


      if (!stage.FireGrasper || this.onCooldown || this.graspers.length === 2)
         return;

      var x = stage.getStage().mouseX;
      var y = stage.getStage().mouseY;
      var p = stage.getStage().localToLocal(x, y, stage);

      x = p.x / Ses.Engine.Scale;
      y = p.y / Ses.Engine.Scale;

      x -= this.ship.body.GetWorldCenter().x;
      y -= this.ship.body.GetWorldCenter().y;

      var grasper = new Ses.Entities.StingGrasper(this.ship, new Ses.b2Vec2(x, y));
      this.graspers.push(grasper);
      stage.addGameObject(grasper);

      var self = this;
      self.onCooldown = true;
      setTimeout(function(){ self.onCooldown = false; }, 1000);
   },

   // delegate
   setOnDieListener: function(callback)
   {
      this.ship.setOnDieListener(callback);
   },

   watch: function()
   {
      this.ship.watch.apply(this.ship, arguments);
   },

   getLifeInPrecetage: function()
   {
      return this.ship.getLifeInPrecetage();
   }

});
