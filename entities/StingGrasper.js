/* jshint bitwise: false */

Ses.Entities.StingGrasper = Ses.Core.Entity.extend({

   // @ship the ship that shoots the grapser
   // @direction vector that shows where to fire grapser
   init: function(ship, direction)
   {
      this.ship = ship;
      var pos = ship.body.GetWorldCenter().Copy();
      // sting apear in front of the ship
      direction.Normalize();
      direction.Multiply( 1.2 );
      pos.Add(direction);

      this.body = Ses.Physic.createCircleObject(
         0.2, //radious
         pos,
         {
            filter: {
               maskBits: ~Ses.Physic.CATEGORY_GRASPER,
               categoryBits: Ses.Physic.CATEGORY_GRASPER
            }
         }
      );
      this.body.SetBullet(true);

      // setup starting velocity
      direction.Multiply( 16 );
      var speed = ship.body.GetLinearVelocity().Copy();
      speed.Add(direction);
      this.body.SetLinearVelocity(speed);

      var self = this;
      Ses.Physic.addOnBeginContactListener(this.body,
         function(collidedBody)
         {
            if (collidedBody.GetUserData())
               if (collidedBody.GetUserData().hookAble)
                  self.attach = true;
         }
      );

      Ses.Physic.addOnPostSolveContactListener(this.body,
            function(collidedBody)
            {
               if (!self.attach)
                  return;

               self.toObjectJoint = Ses.Physic.createWeldJoint(
                  self.body,
                  collidedBody,
                  collidedBody.GetWorldCenter()
               );

               self.toShipJoint = Ses.Physic.createDistanceJoint(
                  ship.body,
                  self.body,
                  ship.body.GetWorldCenter(),
                  self.body.GetWorldCenter()
               );

               self.attach = false;
               Ses.Physic.removeBodyListeners(self.body);
            }
      );

      this.initShape();
   },

   initShape: function()
   {
      this.shape = new createjs.Shape();
      this.shape.graphics
          .beginStroke('#00ff00')
          .drawCircle(0, 0, 0.1 * Ses.Engine.Scale);
   },

   update: function()
   {

      this._super();

      if (this.toShipJoint || this.toObjectJoint)
         return;


      var sp = this.ship.body.GetWorldCenter(),
          tp = this.body.GetWorldCenter().Copy();

      tp.Subtract(sp);

      if (tp.LengthSquared() > 144)
         this.outOfRange = true;
   },

   detach: function()
   {
      if (!this.toObjectJoint || !this.toShipJoint)
         return;

      Ses.Physic.World.DestroyJoint(this.toObjectJoint);
      Ses.Physic.World.DestroyJoint(this.toShipJoint);
   }

});
