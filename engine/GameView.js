Ses.Engine.GameView = Ses.Engine.View.extend({

   useBox2dDebugDraw: false,
   gameObjects: [],
   mapObjectives: [],

   init: function(stage, mapid)
   {
      this._super(stage);
      this.mapid = 0;//mapid;
      Ses.Engine.Factory.gameView = this;

      this.fakeStage = new createjs.Container();
      this.stage.addChild(this.fakeStage);

      //init physic
      Ses.Physic.World = new Ses.b2World(
         new Ses.b2Vec2(0, 0),
         true
      );

      this.initBox2dDebugDraw();
      this.initMap();
      this.initKeyListeners();
   },

   update: function(event)
   {
      //Ses.Physic.processJoints();

      Ses.Physic.World.Step(1/Ses.Engine.FPS, 20, 20);
      Ses.Physic.World.ClearForces();

      Ses.CssUi.update();

      for (var i=0; i<this.gameObjects.length; ++i)
         this.gameObjects[i].update(this.fakeStage);

      this.updateCamera();

      if (this.useBox2dDebugDraw)
         Ses.Physic.World.DrawDebugData();
      else
         this.stage.update();
   },

   initMap: function()
   {
      var objects = Ses.Engine.Maps[this.mapid].objects;
      for(var i=0; i<objects.length; ++i)
      {
         var obj = Ses.Engine.Factory.createObject(objects[i]);
         this.addGameObject(obj);
      }

      var sensor = new Ses.Engine.MapBoundarySensor(10, 10);
      var t = new Ses.Entities.BrokenShip(4, 4);
      this.addGameObject(t);
      //sensor.setOnObjectLeaveListener( function(object)
      //{
      //   Ses.log('object '+object.GetUserData()+' is leaving');
      //});
      this.addGameObject(sensor);

      //this.rock = new Ses.Entities.SpaceRock(640/2, 480/2, 2);
      //var ship =  new Ses.Entities.SpaceShip(40, 40, this.stage);
      //this.ship = new Ses.Entities.SpaceShipWithRope(ship, 6, 10);
      //this.stage.addChild(this.rock.shape);
   },

   initKeyListeners: function()
   {
      Ses.Engine.addKeyListener('D', function(event) {
         if (event.type === 'keydown')
            this.useBox2dDebugDraw = !this.useBox2dDebugDraw;
      });

      Ses.Engine.addKeyListener('Space', function(event) {
         if(event.type === 'keydown')
         {
            this.fakeStage.AttachKeyDown = true;
         }
         else if(event.type === 'keyup')
         {
            this.fakeStage.AttachKeyDown = false;
         }
      });

      var self = this;
      this.stage.addEventListener('stagemousedown', function() {
         self.fakeStage.mousedown = true;
      });
      this.stage.addEventListener('stagemouseup', function() {
         self.fakeStage.mousedown = false;
      });

      Ses.Engine.addMouseScrollListener(function(delta) {
         this.fakeStage.scaleX += delta*0.05;
         this.fakeStage.scaleY += delta*0.05;
      });
   },

   addGameObject: function(object)
   {
      this.gameObjects.push(object);
      if(object.shape)
         this.fakeStage.addChild(object.shape);

      var data = object.body.GetUserData();
      if(data && data.SpaceShip)
         this.cameraFollowObject = object;
   },

   removeGameObject: function(object)
   {
      var i = this.gameObjects.indexOf(object);
      this.gameObjects.splice(i, 1);
      if(object.shape)
         this.fakeStage.removeChild(object.shape);
      Ses.Physic.World.DestroyBody(object.body);
   },

   updateCamera: function()
   {
      if(!this.cameraFollowObject)
         return;

      var o = this.cameraFollowObject.body.GetWorldCenter(),
          s = this.fakeStage;

      s.x = -o.x*Ses.Engine.Scale*s.scaleX;
      s.y = -o.y*Ses.Engine.Scale*s.scaleY;

      var c = this.stage.localToLocal(
            Ses.Engine.ScreenWidth/2,
            Ses.Engine.ScreenHeight/2,
            s);

      var scale = Ses.Engine.Scale;
      s.x += (c.x - o.x*scale)*s.scaleX;
      s.y += (c.y - o.y*scale)*s.scaleY;
   },

   addMapObjective: function(objective, object)
   {
      var objList = this.mapObjectives;
      var self = this;

      var obj = {
         done: false,
         isDone:  function() { return this.done; },
         setDone: function() {
            this.done = true;
            for(var i = 0; i < objList.length; ++i)
            {
               if(!objList[i].isDone())
                  return;
            }
            self.onGameEnd();
         }
      };

      objList.push(obj);

      switch(objective) {

      case 'O_StopInHere':
         Ses.log('Stop in here');
         object.setOnObjectEnterListener(function(x) {
            Ses.log(x.GetLinearVelocity().Length());
         });
         break;

      case 'O_MoveThrough':
         Ses.log('Move through');
         object.setOnObjectEnterListener(function() {
            Ses.log('ktos mnie przelecial');
         });
         break;

      }
   },

   initBox2dDebugDraw: function()
   {
      var debugDraw = new Ses.b2DebugDraw();
      debugDraw.SetDrawScale(Ses.Engine.Scale);
      debugDraw.SetSprite(this.stage.canvas.getContext("2d"));
      debugDraw.SetFillAlpha(0.5);
      debugDraw.SetLineThickness(1.0);
      debugDraw.SetFlags(
         Ses.b2DebugDraw.e_shapeBit | 
         Ses.b2DebugDraw.e_jointBit
      );
      Ses.Physic.World.SetDebugDraw(debugDraw);
   }
});
