Ses.Entities.Grasper = Ses.Core.Entity.extend({

   init: function(lastLinkBody)
   {
      var pos = lastLinkBody.GetWorldCenter();
      this.body = Ses.Physic.createCircleObject(
         0.3, //radious
         { x: pos.x, y: pos.y }
      );
      this.body.SetUserData('grasper');

      var anchor = Ses.Physic.createCircleObject(
         0.2, //radious
         { x: pos.x, y: pos.y }
      );
      Ses.Physic.createRevoluteJoint(
         anchor,
         this.body,
         new Ses.b2Vec2(0, 0),
         new Ses.b2Vec2(0, 0)
      );

      Ses.Physic.createRevoluteJoint(
         lastLinkBody,
         anchor,
         new Ses.b2Vec2(0, 0.25),
         new Ses.b2Vec2(0, -0.5)
      );

      var self = this;
      Ses.Physic.addOnBeginContactListener(this.body,
            function(collidedBody)
            {
               if(collidedBody.GetUserData() === 'rock')
               {
                  Ses.log('collison with rock');
                  Ses.log(collidedBody);
                  self.joint = Ses.Physic.createWeldJoint(
                     self.body,
                     collidedBody,
                     self.body.GetWorldCenter()
                  );
               }
            }
      );

      this.initShape();
   },

   initShape: function()
   {
      var g = new createjs.Graphics();
      g.beginStroke('#ffffff');
      g.setStrokeStyle(1);
      g.drawCircle(0, 0, 0.3*30);
      this.shape = new createjs.Shape(g);
   },

   detach: function()
   {
      if(!this.joint)
         return;

      // Ses.Physic.World.DestroyJoint(this.joint);
      Ses.log('remove joint');
      Ses.Physic.removeJoint(this.joint);
   }

});
