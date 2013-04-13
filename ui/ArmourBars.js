/* global Class */

Ses.Ui.ArmourBars = Class.extend({

   init: function(width, height)
   {
      this.width = width || 100;
      this.height = height || 30;
      this.barWidth = this.width / 2;
      this.barHeigth = this.height / 2;

      this.shape     = new createjs.Container();
      this.shipBar   = new createjs.Shape();
      this.targetBar = new createjs.Shape();
      this.shipText  = new createjs.Text('Ship Armour',
                             this.barHeigth+'px TitilliumText25L400wt', '#ffffff');
      this.targetText= new createjs.Text('',
                             this.barHeigth+'px TitilliumText25L400wt', '#ffffff');

      this.setTargetText('Target Armour');
      this.targetBar.visible = false;
      this.targetText.visible = false;

      this.shape.addChild(this.shipText);

      this.shape.addChild(this.shipBar);
      this.shipBar.y = this.barHeigth;

      this.shape.addChild(this.targetBar);
      this.targetBar.x = this.barWidth;
      this.targetBar.y = this.barHeigth;

      this.shape.addChild(this.targetText);
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
      if (this.targetBar.visible)
         this.shipBar.scaleX = 1;
      else
         this.shipBar.scaleX = 2;

      this.drawBar('rgba(0, 153, 255, 0.2)', 'rgba(0, 153, 255, 0.1)', precentage,
            this.shipBar);
   },

   updateTargetBar: function(precentage)
   {
      this.targetBar.visible = true;
      this.targetText.visible = true;
      this.shipBar.scaleX = 1;

      this.drawBar('rgba(255, 153, 0, 0.2)', 'rgba(255, 153, 0, 0.1)', precentage,
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
      this.targetText.x = this.width - width;
   },

   hide: function()
   {
      this.shape.visible = false;
   }

});
