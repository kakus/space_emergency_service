Ses.Entities.RectangleSensor = Ses.Entities.Sensor.extend({

   Texts: [
      'Dock area. Place here broken ships.',
      'Come closer ...',
      'Good. Dock in ',
      'Broken ship are safety now.'
   ],

   currentText: 0,
   docked: false,

   init: function(x, y, width, height)
   {
      this.body = Ses.Physic.createRectangleSesnor(x, y, width, height,
         {
            filter: {
               maskBits: Ses.Physic.CATEGORY_BROKEN_SHIP
               //categoryBits: Ses.Physic.CATEGORY_BOROKEN_SHIP_SENSOR
            }
         });

      var g = new createjs.Graphics();
      g.beginFill('rgba(0,255,0,0.1)');
      g.beginStroke('rgba(0,255,0,0.51)');

      var pixelWidth  = width *  Ses.Engine.Scale * 2,
          pixelHeight = height * Ses.Engine.Scale * 2;
      g.rect(-pixelWidth/2, -pixelHeight/2, pixelWidth, pixelHeight);
      var arena = new createjs.Shape(g);
      this.shape = new createjs.Container();
      this.shape.addChild(arena);

      this.text = new createjs.Text(this.Texts[this.currentText],
                                    '12px TitilliumText25L400wt', '#00ff00');
      this.text.x = -pixelWidth/2;
      this.text.y = -pixelHeight/2 - this.text.getMeasuredHeight() - 2;
      this.text.cache(0, 0, pixelWidth + 10, 50, 3);
      this.shape.addChild(this.text);
      //createjs.Tween.get(this.shape, {loop:true})
      //   .to({scaleX: 0.95, scaleY: 0.95 }, 300, createjs.Ease.linear)
      //   .to({scaleX: 1, scaleY: 1 }, 300, createjs.Ease.linear);
      //this.shape.shadow = new createjs.Shadow('#00ff00', 0, 0, 4);
   },

   update: function(stage)
   {
      this._super();

      if (!this.docking)
      {
         if (this.docked)
            this.setText(3);
         else
            this.setText(0);
         return;
      }

      var dist = this.docking.GetWorldCenter().Copy();
      dist.Subtract(this.body.GetWorldCenter());
      //if(!stage.debug)
      //{
      //   stage.debug = new createjs.Text('','20px Arial','#ffffff');
      //   stage.getStage().addChild(stage.debug);
      //   stage.debug.x = 200;
      //}

      dist = dist.Length();
      if (dist > 2)
      {
         this.stopCounting();
         this.setText(1);
      }
      else
         this.startCounting();

      //stage.debug.text = dist.toString();

   },

   setText: function(id)
   {
      if (this.currentText === id)
         return;

      this.text.text = this.Texts[id];
      this.currentText = id;
      this.text.updateCache();
   },

   startCounting: function()
   {
      var dockTime = 5;

      if (!this.start)
         this.start = new Date().getTime();

      this.elapsed = new Date().getTime() - this.start;
      this.elapsed /= 1000;
      this.text.text = this.Texts[2] + (dockTime - this.elapsed).toFixed(2);
      this.text.updateCache();
      this.currentText = 2;

      if (this.elapsed > dockTime)
      {
         this.docked = true;
         this.docking = null;
      }
   },

   stopCounting: function()
   {
      this.start = null;
   }

});
