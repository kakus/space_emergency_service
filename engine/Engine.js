/* global Ses */

Ses.Engine = {

   Maps: [],
   currentView: null,

   //BindedKeys: {
   //   'DEBUG_KEY':   68,
   //   'LEFT_ARROW':  37,
   //   'RIGHT_ARROW': 39,
   //   'UP_ARROW':    38,
   //   'DOWN_ARROW':  40,
   //   'SPACE':       32
   //},

   KeyBindings: {

      Space: { code: 32, callbacks: [] },
      D: { code: 68, callbacks: [] },
      Q: { code: 81, callbacks: [] },
      W: { code: 87, callbacks: [] },
      E: { code: 69, callbacks: [] }

   },

   mouseScrollListeners: [],


   init: function(canvas)
   {
      Ses.Engine.FPS = 60;
      // Because Box2d use metric system, we have neet to have a scale pixel/meter
      Ses.Engine.Scale = 30;

      window.onkeydown = this.keyDispatcher;
      window.onkeyup   = this.keyDispatcher;

      // DOMMouseScroll is for mozilla
      if(window.addEventListener)
         window.addEventListener('DOMMouseScroll', this.mouseScrollHandler, false);
      //
      // other web browsers
      window.onmousewheel = this.mouseScrollHandler;

      // Setup box2d, creating shortcuts in our namespace
      Ses.b2Vec2 = Box2D.Common.Math.b2Vec2;
      Ses.b2AABB = Box2D.Collision.b2AABB;
      Ses.b2BodyDef = Box2D.Dynamics.b2BodyDef;
      Ses.b2Body = Box2D.Dynamics.b2Body;
      Ses.b2FixtureDef = Box2D.Dynamics.b2FixtureDef;
      Ses.b2Fixture = Box2D.Dynamics.b2Fixture;
      Ses.b2World = Box2D.Dynamics.b2World;
      Ses.b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;
      Ses.b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;
      Ses.b2DebugDraw = Box2D.Dynamics.b2DebugDraw;

      // resize canvas to the window
      window.onresize = function() {
         canvas.width = window.innerWidth;
         canvas.height = window.innerHeight;
         Ses.Engine.ScreenWidth = canvas.width;
         Ses.Engine.ScreenHeight = canvas.height;
      };
      window.onresize();

      // Setup stage element from createjs, which is root object of all that
      // apear in canvas element
      this.stage = new createjs.Stage(canvas);
      createjs.Ticker.addEventListener('tick', this.updateStage);
      createjs.Ticker.setFPS(Ses.Engine.FPS);
      createjs.Ticker.useRAF = true;

      Ses.log("Engine Created");
   },

   updateStage: function(event)
   {
      if(Ses.Engine.stats)
         Ses.Engine.stats.begin();

      if(Ses.Engine.currentView)
         Ses.Engine.currentView.update(event);

      if(Ses.Engine.stats)
         Ses.Engine.stats.end();
   },

   changeView: function(View) {
      if(this.currentView)
         this.currentView.remove();

      this.currentView = new View(this.stage);
   },

   addKeyListener: function(key, callback)
   {
      if(this.KeyBindings[key])
         this.KeyBindings[key].callbacks.push(callback);
      else
         Ses.err('Key shouldnt be used');
   },

   keyDispatcher: function(event)
   {
      for(var keyname in Ses.Engine.KeyBindings)
      {
         var key = Ses.Engine.KeyBindings[keyname];
         if (key.code === event.keyCode)
         {
            for (var i = 0; i < key.callbacks.length; ++i )
            {
               event.preventDefault();
               key.callbacks[i].call(Ses.Engine.currentView, event);
            }
         }
      }
   },

   mouseScrollHandler: function(event)
   {
      var delta = 0;
      if(event.wheelDelta)
         delta = event.wheelDelta/120;
      else
         delta = -event.detail/3;

      if(delta)
      {
         for(var i = 0; i < Ses.Engine.mouseScrollListeners.length; ++i)
         {
            Ses.Engine.mouseScrollListeners[i].call(
                  Ses.Engine.currentView,
                  delta
            );
            event.preventDefault();
         }
      }

   },

   addMouseScrollListener: function(callback)
   {
      this.mouseScrollListeners.push(callback);
   },

   clearInputListeners: function()
   {
      Ses.Engine.mouseScrollListeners = [];
      for(var key in Ses.Engine.KeyBindings)
         Ses.Engine.KeyBindings[key].callbacks = [];
   },

   loadMaps: function ()
   {
      var xhr = new XMLHttpRequest();
      xhr.open('GET', 'maps/map2.json', false);
      xhr.onload = function () {
         Ses.Engine.processMaps(this.responseText);
      };
      xhr.send();
   },

   processMaps: function(JSON_map)
   {
      var maps = JSON.parse(JSON_map);
      var scaleX = maps.tilewidth;
      var scaleY = maps.tileheight;

      for(var i=0; i<maps.layers.length; ++i)
      {
         var map = maps.layers[i];
         var newMap = {
            name: map.name,
            width: map.width,
            height: map.height,
            objects: []
         };
         this.flatProperties(map, newMap);

         for (var j = 0; j < map.objects.length; ++j)
         {
            var obj = map.objects[j];
            var newObj = {
               name: obj.name,
               width: obj.width / scaleX,
               height: obj.height / scaleY,
               type: obj.type,
               x: obj.x / scaleX + obj.width / scaleX / 2,
               y: obj.y / scaleY + obj.height / scaleY / 2
            };
            this.flatProperties(obj, newObj);
            newMap.objects.push(newObj);
         }

         Ses.Engine.Maps.push(newMap);
         Ses.Menu.addMap(newMap.name, (function(i) {
            return function() {
               Ses.Engine.changeView(Ses.Engine.GameView);
               Ses.Engine.currentView.initMap(i);
            };
         })(i));
      }

      Ses.log(Ses.Engine.Maps);
   },

   showDemo: function()
   {
      Ses.Engine.changeView(Ses.Engine.GameView);
      Ses.Engine.currentView.initMap(0);
   },


   /*
    * This function help with processing map file. Because Tiled (map editor)
    * place all additional properties in properties object we want those
    * properties in our final object without nesting.
    *
    * Example
    * Rock {                  Rock {
    *    x: 12,                  x: 12,
    *    y: 2,                   y: 2,
    *    properties: {     =>    fx: 1,
    *       fx: 1,               fy: 1,
    *       fy: 1             }
    *    }
    * }
    */
   flatProperties: function (srcObj, desObj)
   {
      if (!srcObj.properties)
         return null;

      for (var prop in srcObj.properties)
         desObj[prop] = srcObj.properties[prop];
   },

   /* global Stats */

   showStats: function()
   {
      this.stats = new Stats();
      this.stats.setMode(0);

      this.stats.domElement.style.opacity = 0.3;
      var game_div = document.getElementById('game_div');
      game_div.appendChild(this.stats.domElement);
   }

};
