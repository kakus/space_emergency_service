Ses.Engine.GameView = Ses.Engine.View.extend({

   useBox2dDebugDraw: false,

   init: function(stage)
   {
      this._super(stage);

      //TODO to tez przydalo by sie lepiej rozwiazac
      Ses.Engine.Factory.gameView = this;

      this.gameObjects =         [];
      this.mapObjectives =       [];
      this.conditionsFor2Stars = [];

      this.fakeStage = new createjs.Container();
      this.stage.addChild(this.fakeStage);

      var self = this;
      [this.stage, this.fakeStage].forEach(function(stage) {
         stage.addGameObject =    function(o) { self.addGameObject(o); };
         stage.removeGameObject = function(o) { self.removeGameObject(o); };
         stage.spaceShipOutOfMap = function(o) { self.spaceShipOutOfMap(o); };
      });

      //init physic
      Ses.Physic.World = new Ses.b2World(
         new Ses.b2Vec2(0, 0),
         true
      );

      //this.stage.debug = new createjs.Text('debug', '20px Arial', '#ffffff');
      //this.stage.debug.y = 100;
      //this.stage.debug.x = 10;
      //this.stage.addChild(stage.debug);

      //this.initBox2dDebugDraw();
      this.navigator = new Ses.Engine.Navigator(this.fakeStage);
   },

   update: function(event)
   {
      this.fakeStage.delta = event.delta;
      Ses.Physic.World.Step(event.delta/1000, 10, 10);
      Ses.Physic.World.ClearForces();

      for (var i=0; i<this.gameObjects.length; ++i)
         this.gameObjects[i].update(this.fakeStage);

      this.updateCamera();
      this.navigator.update();

      //if (this.useBox2dDebugDraw)
      //   Ses.Physic.World.DrawDebugData();
      //else
      this.stage.update();


      //this.stage.debug.text = createjs.Ticker.getMeasuredFPS();//((new Date().getTime() - this.start)/1000).toFixed(2);
   },

   onWindowResize: function()
   {
      this.updateUi();
      this.navigator.onWindowResize();
   },

   initMap: function(mapid)
   {
      this.mapid = mapid;
      this.DemoMap = mapid === 0;
      // Demo map, it just display rock while in menu
      if (this.DemoMap)
      {
         this.updateCamera = function() {};
         this.fakeStage.scaleX = this.fakeStage.scaleY = 0.5;
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

      if (this.DemoMap)
         return;

      this.showMapName();
      this.initKeyListeners();
      this.start = new Date().getTime();
      this.lastFrameTime = 0;
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

      //Ses.Engine.addKeyListener('D', function(event) {
      //   if (event.type === 'keydown')
      //      this.useBox2dDebugDraw = !this.useBox2dDebugDraw;
      //});

      Ses.Engine.addKeyListener('Esc', function(event) {
         if (event.type === 'keydown')
            self.onGameOver();
      });

      Ses.Engine.addKeyListener('X', function(event) {
         if (event.type === 'keydown')
            self.onGameRestart();
      });

      this.stage.addEventListener('stagemousedown', function(event) {
         if (event.nativeEvent.button === 0)
            self.fakeStage.StartEngine = true;
      });
      this.stage.addEventListener('stagemouseup', function(event) {
         if (event.nativeEvent.button === 0)
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

      this.ui.shape.y = Ses.Engine.ScreenHeight - this.ui.height - 10; //offset;
      this.ui.shape.x = Ses.Engine.ScreenWidth/2 - this.ui.width/2;
   },

   setupShipUiAndCamera: function(ship)
   {
      this.cameraFollowObject = ship;
      this.navigator.setCenter(ship.body.GetWorldCenter());
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
            object.docking = x;
         });
         object.setOnObjectLeaveListener(function(x) {
            object.docking = null;
         });
         object.watch('docked', function(oldval, newval) {
            if (newval)
               obj.setDone();
         });
         break;

      case 'O_MoveThrough':
         object.setOnObjectEnterListener(function() {
            obj.setDone();
            object.fadeOut(function() { self.removeGameObject(object); });
         });
         break;

      case 'O_CantDie':
         obj.done = true;
         object.setOnDieListener(function() {
            self.onGameOver();
         });
         break;

      case 'O_Track':
         this.navigator.track(object);
         obj.done = true;
         break;
      }
   },

   addScoreCondition: function (cond, object)
   {
      var self = this;
      this.conditionsFor2Stars.push(false);
      var i = this.conditionsFor2Stars.length - 1;

      switch (cond) {

      case 'SC_DeadFor2Stars':
         object.watch('alive', function(oldval, newval) {
            if (newval === false)
               self.conditionsFor2Stars[i] = true;
         });
         break;

      }

   },

   onGameWin: function()
   {
      if (!this.onGameEnd())
         return;

      var stars = 2;
      for (var i = 0; i < this.conditionsFor2Stars.length; ++i)
         if (this.conditionsFor2Stars[i] === false) {
            stars -= 1;
            break;
         }

      var timeFor3Stars = parseFloat(Ses.Engine.Maps[this.mapid].time);
      var playedTime = new Date().getTime() - this.start;
      if ( playedTime < timeFor3Stars * 1000 )
         stars += 1;

      var mapName = Ses.Engine.Maps[this.mapid].name;
      Ses.Menu.showGameWin(mapName, playedTime, stars);
   },

   onGameOver: function()
   {
      var playedTime = new Date().getTime() - this.start;
      var mapName = Ses.Engine.Maps[this.mapid].name;
      if (this.onGameEnd())
         Ses.Menu.showGameOver(mapName, playedTime);
   },

   onGameRestart: function()
   {
      this.onGameEnd();
      Ses.Engine.restartMap();
   },

   onGameEnd: function()
   {
      // while game over screen is being shown, the game is  still runing so we
      // can get callback, and this is our guard so our game end want be called
      // twice
      if(this.fakeStage.gameOver)
         return false;

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

      this.navigator.hide();
      this.ui.hide();
      return true;
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
   },

   showMapName: function()
   {
      var name = new createjs.Text(Ses.Engine.Maps[this.mapid].name,
            '40px TitilliumText25L400wt', '#ffffff');
      name.x = Ses.Engine.ScreenWidth/2 - name.getMeasuredWidth()/2;
      name.y = 50;

      name.shadow = new createjs.Shadow('#0096ff', 0, 0, 8);

      this.stage.addChild(name);
      var self = this;
      createjs.Tween.get(name, {loop:false})
         .wait(1000)
         .to({alpha: 0}, 3000, createjs.Ease.Linear)
         .call(function() {
            self.stage.removeChild(name);
         });
   },

   spaceShipOutOfMap: function(dist)
   {
      this.displayWarning = true;
      // if waring is displayed just return
      if (this.outOfMap) return;

      // create warning text if it doesnt exist
      if (!this.warning)
      {
         var warning = new createjs.Text(
               'You are flying through the endless space, please consider ' +
               'pressing "X" to restart the map.',
               '30px TitilliumText25L400wt', '#ffffff');
         warning.y = 50;

         warning.shadow = new createjs.Shadow('#ff0000', 0, 0, 8);
         this.warning = warning;
         this.stage.addChild(this.warning);
      }

      // center warning on screen
      this.warning.lineWidth = Ses.Engine.ScreenWidth - 100;
      this.warning.x = Ses.Engine.ScreenWidth/2 - this.warning.lineWidth/2;
      this.warning.visible = true;
      this.outOfMap = this.warning;
      var self = this;

      createjs.Tween.get(this.warning, {loop:true})
         .to({alpha: 0.7}, 500, createjs.Ease.Linear)
         .to({alpha: 1},   500, createjs.Ease.Linear)
         .call(function() {
            // we change display settings, so if ship is returns to safe zone we
            // can remove warining. If he doesnt return displayWarining will be
            // set to true more often than we set it to false
            //
            // Seems creepy solution but it works :P
            if (self.displayWarning)
               self.displayWarning = false;
            else
            {
               createjs.Tween.removeTweens(self.warning);
               self.warning.visible = false;
               self.outOfMap = null;
            }
         });

   }
});
