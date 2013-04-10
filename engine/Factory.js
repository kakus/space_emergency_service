Ses.Engine.Factory = {

   factoryMethod: {

      'SpaceRock': function(des) {
         var rock = new Ses.Entities.SpaceRock(des.x, des.y, des.width/2);
         if(des.fx || des.fy)
         {
            var fx = parseFloat(des.fx) || 0,
                fy = parseFloat(des.fy) || 0;

            Ses.log(des);

            //rock.body.ApplyImpulse(new Ses.b2Vec2(fx, fy),
            //                       rock.body.GetWorldCenter());
            rock.body.SetLinearVelocity(new Ses.b2Vec2(fx, fy));
         }
         return rock;
      },

      'SpaceShip': function(des) {
         var ship = new Ses.Entities.SpaceShip(des.x, des.y);
         return ship;
      },

      'SpaceShipWithRope': function(des) {
         return new Ses.Entities.SpaceShipWithRope(this.SpaceShip(des));
      },

      'SpaceShipWithDistanceStick': function(des) {
         return new Ses.Entities.SpaceShipWithDistanceStick(this.SpaceShip(des));
      },

      'CircleSensor': function(des) {
         return new Ses.Entities.CircleSensor(des.x, des.y, des.width/2);
      },

      'RectangleSensor': function(des) {
         return new Ses.Entities.RectangleSensor(
               des.x, des.y, des.width/2, des.height/2);
      },

      'BrokenShip': function(des) {
         var s = new Ses.Entities.BrokenShip(des.x, des.y);
         //return new Ses.Core.EntityWithHitPointsBar(s);
         return s;
      },

      'TextField': function(des) {
         return new Ses.Entities.TextField(des.x, des.y, des.text);
      }
   },

   /* Creates object by the given description, also adds objectives to the
    * current map so we can know when game should end.
    */
   createObject: function(description)
   {
      var obj = this.factoryMethod[description.type](description);

      var isObjective = /O_.*/;
      for (var field in description)
         if (isObjective.test(field))
            this.gameView.addMapObjective(field, obj);

      return obj;
   }
};
