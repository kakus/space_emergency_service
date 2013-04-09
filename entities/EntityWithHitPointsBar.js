/* 
 * Decorator pattern
 *
 * this class is just displaying hitpoints bar
 */
Ses.Core.EntityWithHitPointsBar = Ses.Core.Entity.extend({

   m_drawHitPoints: true,

   init: function(entity)
   {
      this.entity = entity;
      this.body = entity.body;
      var c = new createjs.Container();
      this.shape = c;

      var s = new createjs.Shape();
      this.hitPointsShape = s;
      c.addChild(s);
      c.addChild(entity.shape);
      this.drawHitPoints();
   },

   update: function(stage)
   {
      this.entity.update(stage);

      if (!this.m_drawHitPoints)
         return;

      this.drawHitPoints();
   },

   drawHitPoints: function()
   {
      var h = this.hitPointsShape,
          g = h.graphics,
          s = this.entity.shape,

          hp = this.entity.currentHitPoints,
          maxhp = this.entity.maxHitPoints,

          hitpointsWidth = 40,
          hitpointsHeight = 4;

      if (hp < 0)
         hp = 0;

      g.clear();
      g.beginFill('rgba(0, 153, 255, 0.2)')
         .rect(0, 0, hitpointsWidth, hitpointsHeight);

      g.beginFill('rgba(0, 153, 255, 0.2)')
         .rect(0, 0, hp/maxhp*hitpointsWidth, hitpointsHeight);

      h.x = s.x-hitpointsWidth/2;
      h.y = s.y-40;
   },

   //delegate
   setOnDieListener: function(callback)
   {
      this.entity.setOnDieListener(callback);
   }
});
