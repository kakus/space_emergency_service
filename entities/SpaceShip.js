Ses.Entities.SpaceShip = Ses.Core.Entity.extend({

   init: function(x, y)
   {
      this._super();
      this.armour = Ses.Constans.SpaceShip.Armour;

      this.body = Ses.Physic.createCircleObject(
         Ses.Constans.SpaceShip.Radious,
         { x: x, y: y },
         {
            density: Ses.Constans.SpaceShip.Density,

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
      //TODO sila uderzenia !
      Ses.Physic.addOnPostSolveContactListener(this.body,
            function(contact, impulse) {
               if (impulse.normalImpulses[0] > 3)
                  self.hit(impulse.normalImpulses[0]);
            });

      this.initShape();
   },

   initShape: function()
   {
      var g = new createjs.Graphics();
      var width = 44  ;
      var height = 43 ;

      g.beginStroke('#ffffff');
      g.setStrokeStyle(1);
      g.rect(-width/2, -height/2, width, height);

      this.shape = new createjs.Shape(g);

      var oldDraw = this.shape.draw;
      this.shape.draw = this.getCustomDrawFunction(-width/2, -height/2);
      //this.shape.x = -width/2;
      //this.shape.y = -height/2;
      //this.shape.cache(-width/2, -height/2, width, height, 1);
      //this.shape.draw = oldDraw;
   },

   update: function(fakeStage)
   {
      this._super();

      if(fakeStage.gameOver)
         return;

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
   },

   draw: function(ctx)
   {
      // Auto generated code !
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0,0);
      ctx.lineTo(43.977,0);
      ctx.lineTo(43.977,42.616);
      ctx.lineTo(0,42.616);
      ctx.closePath();
      ctx.clip();
      ctx.strokeStyle = 'rgba(0,0,0,0)';
      ctx.lineCap = 'butt';
      ctx.lineJoin = 'miter';
      ctx.miterLimit = 4;
      ctx.save();
      ctx.restore();
      ctx.save();
      ctx.fillStyle = "rgba(0, 0, 0, 0)";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1;
      ctx.lineJoin = "bevel";
      ctx.miterLimit = 4;
      ctx.beginPath();
      ctx.moveTo(22,21.353);
      ctx.bezierCurveTo(17.118000000000002,21.461000000000002,12.599,17.481,12.08,12.627);
      ctx.bezierCurveTo(11.618,9.352,12.974,5.985,15.407,3.7827);
      ctx.bezierCurveTo(15.912,2.8436000000000003,18.523,1.27,16.749,0.5429000000000004);
      ctx.bezierCurveTo(13.985999999999999,0.3338200000000004,11.54,2.0017000000000005,9.3441,3.4711000000000003);
      ctx.bezierCurveTo(3.0759999999999996,7.9451,-0.3846000000000007,15.9611,0.672699999999999,23.5921);
      ctx.bezierCurveTo(1.020719999999999,25.421999999999997,1.032369999999999,27.779799999999998,2.889699999999999,28.7941);
      ctx.bezierCurveTo(4.825599999999999,29.9509,6.368199999999999,27.6948,6.238099999999999,25.916);
      ctx.bezierCurveTo(6.977759999999999,22.8525,10.9782,21.355600000000003,13.5926,23.0777);
      ctx.bezierCurveTo(16.2367,24.5635,16.8158,28.5402,14.700999999999999,30.7157);
      ctx.bezierCurveTo(13.0765,32.5841,9.983999999999998,32.9244,8.046799999999998,31.35111);
      ctx.bezierCurveTo(6.345499999999998,30.49017,4.781399999999998,32.36991,5.247899999999998,33.99511);
      ctx.bezierCurveTo(5.6933599999999975,35.63661,7.405999999999998,36.446009999999994,8.523899999999998,37.607409999999994);
      ctx.bezierCurveTo(11.447899999999997,39.966609999999996,15.017599999999998,41.553509999999996,18.725899999999996,42.106109999999994);
      ctx.bezierCurveTo(20.864499999999996,42.111909999999995,20.015099999999997,39.665409999999994,19.113609999999994,38.67170999999999);
      ctx.bezierCurveTo(18.010609999999993,37.48170999999999,16.917609999999993,36.07570999999999,16.999609999999993,34.35270999999999);
      ctx.bezierCurveTo(16.946339999999992,31.659809999999986,19.304109999999994,29.299709999999987,21.999609999999993,29.352709999999988);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.save();
      ctx.transform(-1,0,0,1,43.976714,0);
      ctx.translate(0,0);
      ctx.translate(0,0);
      ctx.save();
      ctx.fillStyle = "rgba(0, 0, 0, 0)";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1;
      ctx.lineJoin = "bevel";
      ctx.miterLimit = 4;
      ctx.beginPath();
      ctx.moveTo(22,21.353);
      ctx.bezierCurveTo(17.118000000000002,21.461000000000002,12.599,17.481,12.08,12.627);
      ctx.bezierCurveTo(11.618,9.352,12.974,5.985,15.407,3.7827);
      ctx.bezierCurveTo(15.912,2.8436000000000003,18.523,1.27,16.749,0.5429000000000004);
      ctx.bezierCurveTo(13.985999999999999,0.3338200000000004,11.54,2.0017000000000005,9.3441,3.4711000000000003);
      ctx.bezierCurveTo(3.0759999999999996,7.9451,-0.3846000000000007,15.9611,0.672699999999999,23.5921);
      ctx.bezierCurveTo(1.020719999999999,25.421999999999997,1.032369999999999,27.779799999999998,2.889699999999999,28.7941);
      ctx.bezierCurveTo(4.825599999999999,29.9509,6.368199999999999,27.6948,6.238099999999999,25.916);
      ctx.bezierCurveTo(6.977759999999999,22.8525,10.9782,21.355600000000003,13.5926,23.0777);
      ctx.bezierCurveTo(16.2367,24.5635,16.8158,28.5402,14.700999999999999,30.7157);
      ctx.bezierCurveTo(13.0765,32.5841,9.983999999999998,32.9244,8.046799999999998,31.35111);
      ctx.bezierCurveTo(6.345499999999998,30.49017,4.781399999999998,32.36991,5.247899999999998,33.99511);
      ctx.bezierCurveTo(5.6933599999999975,35.63661,7.405999999999998,36.446009999999994,8.523899999999998,37.607409999999994);
      ctx.bezierCurveTo(11.447899999999997,39.966609999999996,15.017599999999998,41.553509999999996,18.725899999999996,42.106109999999994);
      ctx.bezierCurveTo(20.864499999999996,42.111909999999995,20.015099999999997,39.665409999999994,19.113609999999994,38.67170999999999);
      ctx.bezierCurveTo(18.010609999999993,37.48170999999999,16.917609999999993,36.07570999999999,16.999609999999993,34.35270999999999);
      ctx.bezierCurveTo(16.946339999999992,31.659809999999986,19.304109999999994,29.299709999999987,21.999609999999993,29.352709999999988);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.restore();
      ctx.restore();
      // end auto generated code
   }
});
