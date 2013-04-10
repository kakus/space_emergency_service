/* global Class */

Ses.Entities.TextField = Class.extend({

   init: function(x, y, text)
   {
      this.shape = new createjs.Text(text, '16px TitilliumText25L400wt', 'rgba(255,255,255,0.5)');
      this.shape.lineWidth = 300;
      this.shape.cache(0, 0, // 300, 50, 2);
         this.shape.getMeasuredWidth()+10, this.shape.getMeasuredHeight()+10, 3);
      this.shape.x = x*Ses.Engine.Scale;
      this.shape.y = y*Ses.Engine.Scale;
   }

});
