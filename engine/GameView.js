Ses.Engine.GameView = Ses.Engine.View.extend({

   useBox2dDebugDraw: false,

   init: function(stage)
   {
      this._super(stage);

      //TODO to tez przydalo by sie lepiej rozwiazac
      Ses.Engine.Factory.gameView = this;

      this.gameObjects = [];
      this.mapObjectives = [];

      this.fakeStage = new createjs.Container();
      this.stage.addChild(this.fakeStage);
      var self = this;
      this.fakeStage.addGameObject = function(object){
         self.addGameObject(object);
      };
      this.fakeStage.removeGameObject = function(object){
         self.removeGameObject(object);
      };

      //init physic
      Ses.Physic.World = new Ses.b2World(
         new Ses.b2Vec2(0, 0),
         true
      );

      this.initBox2dDebugDraw();
   },

   update: function(event)
   {
      Ses.Physic.World.Step(1/Ses.Engine.FPS, 10, 10);
      Ses.Physic.World.ClearForces();

      for (var i=0; i<this.gameObjects.length; ++i)
         this.gameObjects[i].update(this.fakeStage);

      this.updateCamera();

      if (this.useBox2dDebugDraw)
         Ses.Physic.World.DrawDebugData();
      else
         this.stage.update();
   },

   onWindowResize: function()
   {
      this.updateUi();
   },

   initMap: function(mapid)
   {
      // Demo map, it just display rock while in menu
      if (mapid === 0)
      {
         this.updateCamera = function() {};
         this.fakeStage.scaleX = this.fakeStage.scaleY = 0.5;
         this.DemoMap = true;
      }
      if (Ses.Engine.Maps[mapid].scale)
      {
         this.fakeStage.scaleX = parseFloat(Ses.Engine.Maps[mapid].scale);
         this.fakeStage.scaleY = parseFloat(Ses.Engine.Maps[mapid].scale);
      }

      var objects = Ses.Engine.Maps[mapid].objects;
      for(var i=0; i<objects.length; ++i)
      {
         var obj = Ses.Engine.Factory.createObject(objects[i]);
         this.addGameObject(obj);
      }

      if (mapid !== 0)
         this.initKeyListeners();
   },

   initKeyListeners: function()
   {
      var bindings = {
         'Q' : 'StartEngine',
         'W' : 'FireGrasper',
         'E' : 'RemoveGraspers',
      };

      var self = this;
      for (var key in bindings)
      {
         (function(key) {
            Ses.Engine.addKeyListener(key, function(event)
            {
               if (event.type === 'keydown')
                  self.fakeStage[bindings[key]] = true;
               else if(event.type === 'keyup')
                  self.fakeStage[bindings[key]] = false;
            });
         })(key);
      }

      Ses.Engine.addKeyListener('D', function(event) {
         if (event.type === 'keydown')
            this.useBox2dDebugDraw = !this.useBox2dDebugDraw;
      });

      this.stage.addEventListener('stagemousedown', function() {
         self.fakeStage.StartEngine = true;
      });
      this.stage.addEventListener('stagemouseup', function() {
         self.fakeStage.StartEngine = false;
      });

      Ses.Engine.addMouseScrollListener(function(delta) {

         this.fakeStage.scaleX += delta*0.05;

         if(this.fakeStage.scaleX < 0.2)
            this.fakeStage.scaleX = 0.2;
         if(this.fakeStage.scaleX > 10)
            this.fakeStage.scaleX = 10;

         this.fakeStage.scaleY += delta*0.05;

         if(this.fakeStage.scaleY < 0.2)
            this.fakeStage.scaleY = 0.2;
         if(this.fakeStage.scaleY > 10)
            this.fakeStage.scaleY = 10;
      });
   },

   addGameObject: function(object)
   {
      if (object.body)
         this.gameObjects.push(object);
      if(object.shape)
         this.fakeStage.addChild(object.shape);

      if (this.DemoMap || !object.body)
         return;

      var data = object.body.GetUserData();
      if(data && data.SpaceShip)
         this.setupShipUiAndCamera(object);
      else if(data && data.BrokenShip)
         // TODO better solution :/
         this.setupTargetUi(object);
   },

   removeGameObject: function(object)
   {
      var i = this.gameObjects.indexOf(object);
      this.gameObjects.splice(i, 1);
      if(object.shape)
         this.fakeStage.removeChild(object.shape);
      Ses.Physic.World.DestroyBody(object.body);
   },

   setupUi: function()
   {
      this.ui = new Ses.Ui.ArmourBars(300, 30);
      this.stage.addChild(this.ui.shape);
      this.updateUi();
   },

   updateUi: function()
   {
      if (!this.ui)
         return;

      this.ui.shape.y = Ses.Engine.ScreenHeight - this.ui.height - 10 //offset;
      this.ui.shape.x = Ses.Engine.ScreenWidth/2 - this.ui.width/2;
   },

   setupShipUiAndCamera: function(ship)
   {
      this.cameraFollowObject = ship;
      if (!this.ui)
         this.setupUi();

      this.ui.updateShipBar(ship.getLifeInPrecetage());

      var self = this;
      ship.watch('currentHitPoints', function( oldval, newval) {
         var precentage = 0;
         if (newval > 0)
            precentage = newval / ship.maxHitPoints;

         self.ui.updateShipBar(precentage);
      });
   },

   setupTargetUi: function(target)
   {
      if (!this.ui)
         this.setupUi();

      this.ui.updateTargetBar(target.getLifeInPrecetage());
      var self = this;
      target.watch('currentHitPoints', function(oldval, newval) {
         var precentage = 0;
         if (newval > 0)
            precentage = newval / target.maxHitPoints;

         self.ui.updateTargetBar(precentage);
      });
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
            self.onGameWin();
         }
      };

      objList.push(obj);

      switch(objective) {

      case 'O_StopInHere':
         object.setOnObjectEnterListener(function(x) {
            Ses.log(x.GetLinearVelocity().Length());
         });
         break;

      case 'O_MoveThrough':
         object.setOnObjectEnterListener(function() {
            obj.setDone();
            self.removeGameObject(object);
         });
         break;

      case 'O_CantDie':
         obj.done = true;
         object.setOnDieListener(function() {
            self.onGameOver();
         });

      }
   },

   onGameWin: function()
   {
      this.onGameEnd();
      Ses.Menu.showGameWin();
   },

   onGameOver: function()
   {
      this.onGameEnd();
      Ses.Menu.showGameOver();
   },

   onGameEnd: function()
   {
      // while game over screen is being shown, the game is  still runing so we
      // can get callback, and this is our guard 
      if(this.fakeStage.gameOver)
         return;

      // cleanup objectives
      this.mapObjectives = [];

      this.stage.removeAllEventListeners();
      Ses.Engine.clearInputListeners();

      var self = this;
      this.fakeStage.gameOver = true;
      this.update = function() {
         Ses.Physic.World.Step(1/(Ses.Engine.FPS*16), 2, 2);
         Ses.Physic.World.ClearForces();
         for (var i=0; i<self.gameObjects.length; ++i)
            self.gameObjects[i].update(self.fakeStage);
         self.stage.update();
      };

      Ses.CssUi.hideSipStats();
   },

   // This function is called when we change levels, so it clean up this map.
   remove: function()
   {
      for(var i = 0; i < this.gameObjects.length; ++i)
         this.removeGameObject(this.gameObjects[i]);

      // destroy world
      Ses.Physic.destroyWorld();

      this.stage.removeAllChildren();
      this.stage.clear();

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
