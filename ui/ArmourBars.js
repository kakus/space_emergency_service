/* global Class */

Ses.Ui.ArmourBars = Class.extend({

   init: function(width, height)
   {
      this.width = width || 100;
      this.height = height || 30;
      this.barWidth = this.width;
      this.barWidth /= 2;
      this.barHeigth = this.height;
      this.barHeigth /= 2;

      this.shape     = new createjs.Container();
      this.shipBar   = new createjs.Shape();
      this.targetBar = new createjs.Shape();
      this.shipText  = new createjs.Text('Ship Armour',
                                         this.barHeigth+'px TitilliumText25L400wt', '#ffffff');
      this.targetText= new createjs.Text('Target Armour',
                                         this.barHeigth+'px TitilliumText25L400wt', '#ffffff');

      this.shape.addChild(this.shipText);

      this.shape.addChild(this.shipBar);
      this.shipBar.y = this.barHeigth;

      this.shape.addChild(this.targetBar);
      this.targetBar.x = this.barWidth;
      this.targetBar.y = this.barHeigth;

      this.shape.addChild(this.targetText);
      this.targetText.x = this.barWidth;

      //this.shape.cache(0, 0, width, heigth);
   },

   drawBar: function(foregroundColor, backgroundColor, precentage, shape)
   {
      shape.graphics
         
         .clear()
         .beginFill(backgroundColor)
         .rect(0, 0, this.barWidth, this.barHeigth)
         .beginFill(foregroundColor)
         .rect(0, 0, this.barWidth*precentage, this.barHeigth);
   },

   updateShipBar: function(precentage)
   {
      this.drawBar('rgba(0, 153, 255, 0.2)', 'rgba(0, 153, 255, 0.2)', precentage,
            this.shipBar);
   },

   updateTargetBar: function(precentage)
   {
      this.drawBar('rgba(255, 153, 0, 0.2)', 'rgba(255, 153, 0, 0.2)', precentage,
            this.targetBar);
   },

   setShipText: function(text)
   {
      this.shipText.text = text;
   },

   setTargetText: function(text)
   {
      this.targetText.text = text;
      var width = this.targetText.getMeasuredWidth();
      this.targetText.x = this.barWidth*2 - width;
   }

});
