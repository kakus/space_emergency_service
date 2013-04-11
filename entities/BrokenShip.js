Ses.Entities.BrokenShip = Ses.Core.Entity.extend({

   init: function(x, y)
   {
      this.body = Ses.Physic.createRectangleObject(
         66/Ses.Engine.Scale/2,
         113/Ses.Engine.Scale/2,
         { x: x, y: y },
         {
            filter: {
               categoryBits: Ses.Physic.CATEGORY_WORLD &&
                             Ses.Physic.CATEGORY_BROKEN_SHIP,
               maskBits: Ses.Physic.WORLD_MASK
            }
         }
      );

      var s = new createjs.Shape();
      s.graphics.beginFill("#FF0000").rect(0, 0, 75, 100);

      var oldDraw = s.draw;
      // here we switch draw funciton to the one generated from svg file
      s.draw = this.getCustomDrawFunction(-66/2, -113/2);
      //s.x = -166/2;
      //s.y = -113/2;
      // draw our model on cache
      //s.cache(-66/2, -113/2, 66, 113, 2);
      // and now we can back to old drawing function
      //s.draw = oldDraw;

      this.shape = s;
      this.body.SetUserData({
         hookAble: true,
         BrokenShip: true
      });
      this.setKillable();

      switch (Ses.Engine.Graphics) {
      case 'low':
         this.shape.cache(-66/2, -113/2, 66, 113, 1);
         s.draw = oldDraw;
         break;
      }
   },

   draw: function(ctx)
   {
      // Auto generated code !
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(0,0);
      ctx.lineTo(66.161,0);
      ctx.lineTo(66.161,113.06);
      ctx.lineTo(0,113.06);
      ctx.closePath();
      ctx.clip();
      ctx.strokeStyle = 'rgba(0,0,0,0)';
      ctx.lineCap = 'butt';
      ctx.lineJoin = 'miter';
      ctx.miterLimit = 4;
      ctx.save();
      ctx.restore();
      ctx.save();
      ctx.fillStyle = "rgba(0, 0, 0, 0)";
      ctx.strokeStyle = "#ffffff";
      ctx.translate(0.14570164,-0.28014181);
      ctx.save();
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(33.938,0.84934);
      ctx.bezierCurveTo(12.542000000000002,-0.60006,-5.1739999999999995,22.693340000000003,1.951000000000004,42.923339999999996);
      ctx.bezierCurveTo(4.6185000000000045,54.51434,16.139000000000003,61.866339999999994,25.500000000000004,65.73133999999999);
      ctx.bezierCurveTo(25.507300000000004,67.34593999999998,25.956500000000002,69.21083999999999,23.924000000000003,69.67053999999999);
      ctx.bezierCurveTo(21.116000000000003,71.35553999999999,18.308000000000003,73.04054,15.500000000000004,74.72453999999999);
      ctx.bezierCurveTo(15.241000000000003,70.47554,16.642000000000003,65.16654,12.608000000000004,62.25453999999999);
      ctx.bezierCurveTo(8.286400000000004,58.91153999999999,0.7440000000000033,61.643539999999994,0.5430000000000046,67.41054);
      ctx.bezierCurveTo(0.4430400000000046,82.71154,0.5283300000000046,98.01653999999999,0.4999700000000046,113.32054);
      ctx.bezierCurveTo(0.04649000000000458,107.74054,6.676670000000005,104.10054,11.400970000000004,106.55054);
      ctx.bezierCurveTo(14.210970000000005,107.04223999999999,15.733070000000005,112.29834,15.499770000000005,112.79303999999999);
      ctx.lineTo(15.499770000000005,80.56804);
      ctx.bezierCurveTo(18.833070000000006,78.56804,22.166470000000004,76.56804,25.499770000000005,74.56804);
      ctx.bezierCurveTo(25.587060000000005,79.08633999999999,25.316480000000006,83.63083999999999,25.651620000000005,88.13104);
      ctx.bezierCurveTo(27.293920000000004,94.98913999999999,38.70562,94.98913999999999,40.347620000000006,88.13104);
      ctx.bezierCurveTo(40.67985000000001,83.65124,40.414190000000005,79.12804,40.49947000000001,74.63004);
      ctx.bezierCurveTo(43.83277000000001,76.63004,47.16617000000001,78.63004,50.49947000000001,80.63004);
      ctx.lineTo(50.49947000000001,113.31804);
      ctx.bezierCurveTo(50.04599000000001,107.74444,56.67617000000001,104.09754,61.40047000000001,106.55044);
      ctx.bezierCurveTo(64.21047000000002,107.04213999999999,65.73257000000001,112.29823999999999,65.49927000000001,112.79293999999999);
      ctx.bezierCurveTo(65.47027000000001,97.66893999999999,65.55627000000001,82.53893999999998,65.45627,67.41293999999999);
      ctx.bezierCurveTo(65.24827,61.051939999999995,56.29227,58.60493999999999,52.33827,63.19393999999999);
      ctx.bezierCurveTo(49.547270000000005,66.42594,50.773270000000004,70.89793999999999,50.49927,74.78993999999999);
      ctx.bezierCurveTo(47.16597,72.78993999999999,43.832570000000004,70.78993999999999,40.49927,68.78993999999999);
      ctx.bezierCurveTo(40.81219,66.65033999999999,39.183370000000004,63.05303999999999,43.618770000000005,64.00823999999999);
      ctx.bezierCurveTo(63.359770000000005,57.97623999999999,72.11277000000001,31.576239999999984,59.81177000000001,14.97323999999999);
      ctx.bezierCurveTo(54.07977000000001,6.489539999999989,44.172770000000014,1.0832399999999893,33.93677000000001,0.8522399999999895);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.save();
      ctx.lineWidth = 1.0808545351028442;
      ctx.lineJoin = "round";
      ctx.miterLimit = 4;
      ctx.beginPath();
      ctx.moveTo(47.023,32.362);
      ctx.bezierCurveTo(47.38241,41.623000000000005,37.095400000000005,48.979,28.448000000000004,45.645);
      ctx.bezierCurveTo(19.886300000000006,43.0642,16.082000000000004,31.628000000000004,21.400700000000004,24.436000000000003);
      ctx.bezierCurveTo(26.333200000000005,16.589600000000004,38.97670000000001,16.326600000000003,44.231700000000004,23.961130000000004);
      ctx.bezierCurveTo(46.0352,26.364030000000003,47.029700000000005,29.357930000000003,47.02310000000001,32.36193);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      ctx.restore();
      ctx.restore();
      //End auto generated code
   }

});
