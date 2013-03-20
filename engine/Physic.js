Ses.Physic = {

   bodyDef:          new Box2D.Dynamics.b2BodyDef(),
   fixtureDef:       new Box2D.Dynamics.b2FixtureDef(),
   revoluteJointDef: new Box2D.Dynamics.Joints.b2RevoluteJointDef(),
   weldJointDef:     new Box2D.Dynamics.Joints.b2WeldJointDef(),


   onBeginContactCallbacks: [],
   onEndContactCallbacks: [],

   jointsToCreate: [],
   jointsToRemove: [],

   CATEGORY_SHIP : 0x01,
   CATEGORY_JET_PARTICLE : 0x02,
   CATEGORY_WORLD : 0xff,

   SHIP_MASK :         ~0x02, // Everything that is not jet particle
   JET_PARTICLE_MASK : ~0x01 & ~0x02, // Everything that is not ship and not particle

   setupFixtureDef: function(fixtureDef, specification)
   {
      if(specification)
      {
         for(var field in specification)
         {
            if(typeof specification[field] === 'object')
            {
               for(var field_2 in specification[field])
                  fixtureDef[field][field_2] = specification[field][field_2];

            }
            else
               fixtureDef[field] = specification[field];
         }
      }
      else
      {
         fixtureDef.density = 1;
         fixtureDef.friction = 0.5;
      }
   },

   createBody: function(fixtureDef)
   {
      var body = Ses.Physic.World.CreateBody(this.bodyDef);
      body.CreateFixture(fixtureDef);
      return body;
   },

   createStaticBody: function(fixtureDef, position)
   {
      this.bodyDef.type = Box2D.Dynamics.b2Body.b2_staticBody;
      this.bodyDef.position.Set(position.x, position.y);
      return this.createBody(fixtureDef);
   },

   createDynamicBody: function(fixtureDef, position)
   {
      this.bodyDef.type = Box2D.Dynamics.b2Body.b2_dynamicBody;
      this.bodyDef.position.Set(position.x, position.y);
      return this.createBody(fixtureDef);
   },

   createRectangleObject: function(width, height, position, specification)
   {
      var fixtureDef = new Box2D.Dynamics.b2FixtureDef();
      this.setupFixtureDef(fixtureDef, specification);

      fixtureDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
      fixtureDef.shape.SetAsBox(width, height);
      return this.createDynamicBody(fixtureDef, position);
   },

   createCircleObject: function(radious, position, specification)
   {
      var fixtureDef = new Box2D.Dynamics.b2FixtureDef();
      this.setupFixtureDef(fixtureDef, specification);
      fixtureDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radious);
      return this.createDynamicBody(fixtureDef, position);
   },

   createRevoluteJoint: function(bodyA, bodyB, vecA, vecB)
   {
      this.revoluteJointDef.localAnchorA.Set(vecA.x, vecA.y);
      this.revoluteJointDef.localAnchorB.Set(vecB.x, vecB.y);
      this.revoluteJointDef.bodyA = bodyA;
      this.revoluteJointDef.bodyB = bodyB;
      Ses.Physic.World.CreateJoint(this.revoluteJointDef);
   },

   createWeldJoint: function(bodyA, bodyB, vec)
   {
      var weldJointDef = new Box2D.Dynamics.Joints.b2WeldJointDef();
      weldJointDef.Initialize(bodyA, bodyB, vec);
      //this.jointsToCreate.push(weldJointDef);
      return Ses.Physic.World.CreateJoint(weldJointDef);
   },

   initContactListener: function()
   {
      var contactListener = new Box2D.Dynamics.b2ContactListener();
      contactListener.BeginContact = this.onBeginContactHandler;
      contactListener.EndContact =   this.onEndContactHandler;
      Ses.Physic.World.SetContactListener(contactListener);
      this.contactListener = contactListener;
   },

   onBeginContactHandler: function(contact)
   {
      var bodyA = contact.GetFixtureA().GetBody();
      var bodyB = contact.GetFixtureB().GetBody();

      for(var i = 0; i < Ses.Physic.onBeginContactCallbacks.length; ++i)
      {
         var body = Ses.Physic.onBeginContactCallbacks[i].body;
         var callback = Ses.Physic.onBeginContactCallbacks[i].callback;

         if( bodyA === body)
            callback.call(undefined, bodyB);
         else if( bodyB === body )
            callback.call(undefined, bodyA);
      }
   },

   onEndContactHandler: function(contact)
   {
      var bodyA = contact.GetFixtureA().GetBody();
      var bodyB = contact.GetFixtureB().GetBody();

      for(var i = 0; i < Ses.Physic.onEndContactCallbacks.length; ++i)
      {
         var body = Ses.Physic.onEndContactCallbacks[i].body;
         var callback = Ses.Physic.onEndContactCallbacks[i].callback;

         if( bodyA === body)
            callback.call(undefined, bodyB);
         else if( bodyB === body )
            callback.call(undefined, bodyA);
      }
   },

   addOnBeginContactListener: function(body, callback)
   {
      if (!this.contactListener)
         this.initContactListener();

      this.onBeginContactCallbacks.push({ body: body, callback: callback });
   },

   addOnEndContactListener: function(body, callback)
   {
      if (!this.contactListener)
         this.initContactListener();

      this.onEndContactCallbacks.push({ body: body, callback: callback });
   },

   createRectangleSesnor: function(x, y, width, height)
   {
      var fixtureDef = new Box2D.Dynamics.b2FixtureDef();
      fixtureDef.shape = new Box2D.Collision.Shapes.b2PolygonShape();
      fixtureDef.shape.SetAsBox(width, height);
      fixtureDef.isSensor = true;
      return this.createStaticBody(fixtureDef, new Ses.b2Vec2(x, y));
   },

   createCircleSesnor: function(x, y, radious)
   {
      var fixtureDef = new Box2D.Dynamics.b2FixtureDef();
      fixtureDef.shape = new Box2D.Collision.Shapes.b2CircleShape(radious);
      fixtureDef.isSensor = true;
      return this.createStaticBody(fixtureDef, new Ses.b2Vec2(x, y));
   },


   //TODO all underneath

   removeJoint: function(joint)
   {
      ///this.jointsToRemove.push(joint);
      Ses.Physic.World.DestroyJoint(joint);
   },

   processJoints: function()
   {
      for(var i = 0; i < this.jointsToCreate.length; ++i)
      {
         var def = this.jointsToCreate[i];
         Ses.Physic.World.CreateJoint(def);
      }
      this.jointsToCreate = [];

      for(var i = 0; i < this.jointsToRemove.length; ++i)
      {
         var def = this.jointsToRemove[i];
         Ses.Physic.World.RemoveJoint(def);
      }
      this.jointsToRemove= [];
   }
};
