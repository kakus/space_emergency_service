Ses.Entities.SpaceShip = Ses.Core.Entity.extend({

   init: function(x, y)
   {
      this._super();
      this.armour = Ses.Constans.SpaceShip.Armour;

      this.body = Ses.Physic.createRectangleObject(
         Ses.Constans.SpaceShip.Width,
         Ses.Constans.SpaceShip.Height,
         { x: x, y: y },
         {
            density: 4,

            filter: {
               maskBits: Ses.Physic.SHIP_MASK,
               categoryBits: Ses.Physic.CATEGORY_SHIP
            }
         }
      );
      this.body.SetBullet(true);

      this.body.SetUserData({

         SpaceShip: true

      });

      var self = this;
      Ses.Physic.addOnPostSolveContactListener(this.body,
            function(contact, impulse) {
               if (impulse.normalImpulses[0] > 10)
                 self.armour -= impulse.normalImpulses[0];
            });

      this.initShape();
   },

   initShape: function()
   {
      var g = new createjs.Graphics();
      var width = Ses.Constans.SpaceShip.Width * 2 * Ses.Engine.Scale;
      var height = Ses.Constans.SpaceShip.Height * 2 * Ses.Engine.Scale;

      g.beginStroke('#ffffff');
      g.setStrokeStyle(1);
      g.rect(-width/2, -height/2, width, height);

      this.shape = new createjs.Shape(g);
   },

   update: function(fakeStage)
   {
      this._super();

      var x = fakeStage.getStage().mouseX;
      var y = fakeStage.getStage().mouseY;
      var p = fakeStage.getStage().localToLocal(x, y, fakeStage);

      x = p.x / Ses.Engine.Scale;
      y = p.y / Ses.Engine.Scale;

      x -= this.body.GetWorldCenter().x;
      y -= this.body.GetWorldCenter().y;

      this.body.SetAngle(Math.atan2(y, x) + Math.PI/2);

      if(fakeStage.mousedown)
         this.startEngine();
   },

   startEngine: function()
   {
      var power = 0.5;

      var impulse = new Ses.b2Vec2(
               Math.cos(this.body.GetAngle() - Math.PI/2) * power,
               Math.sin(this.body.GetAngle() - Math.PI/2) * power);


      this.body.ApplyImpulse(impulse, this.body.GetWorldCenter());

      var speed = this.body.GetLinearVelocity();
      var particle = this.createJetParticle();
      particle.body.SetLinearVelocity(speed);
      impulse.NegativeSelf();
      impulse.x *= 0.03+0.05*Math.random();
      impulse.y *= 0.03+0.05*Math.random();
      particle.body.ApplyImpulse(impulse, particle.body.GetWorldCenter());
   },

   createJetParticle: function(stage)
   {
      var shipPos = this.body.GetWorldCenter();
      var particle = new Ses.Entities.JetExhaustParticle(
            shipPos.x,
            shipPos.y,
            0.05 + Math.random()*0.15
      );

      //TODO obejsc to ! dodawanie do elementu statku albo inaczej do sceny
      Ses.Engine.currentView.addGameObject(particle);
      particle.startFading(1000);

      setTimeout(function() {
         Ses.Engine.currentView.removeGameObject(particle);
      }, 1000);

      return particle;
   }
});
