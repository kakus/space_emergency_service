/* global _gaq */

Ses.Menu = (function() {

   var mainMenu = document.getElementById('main-menu'),
       levels = document.getElementById('level-entries'),
       gameOver = document.getElementById('game-end'),
       canvas   = document.getElementById('canvas'),
       restartButton = document.getElementById('restart'),
       graphicsQuality = document.getElementById('graphics-quality'),
       backToMenu = document.getElementById('back-to-menu');

   // buttons
   graphicsQuality.innerHTML = 'graphics: high';
   graphicsQuality.type = 2;
   graphicsQuality.onclick = function() {

      this.type = ( ++this.type ) % 3;
      switch (graphicsQuality.type) {
      case 0:
         graphicsQuality.innerHTML = 'graphics: low';
         Ses.Engine.Graphics = 'low';
         break;

      case 1:
         graphicsQuality.innerHTML = 'graphics: medium';
         Ses.Engine.Graphics = 'medium';
         break;

      case 2:
         graphicsQuality.innerHTML = 'graphics: high';
         Ses.Engine.Graphics = 'high';
         break;
      }
   };

   restartButton.onclick = function() {
      gameOver.style.display = 'none';
      Ses.Engine.restartMap();
   };

   backToMenu.onclick = function()
   {
      gameOver.style.display = 'none';
      mainMenu.style.display = 'block';
   };

   return {

      addMap: function(mapName, callback)
      {
         var mapLabel = document.createElement('div');
         mapLabel.className = 'ui button';
         mapLabel.innerHTML = mapName;
         mapLabel.onclick = function() {
            mainMenu.style.display = 'none';
            callback.call();
         };

         levels.appendChild(mapLabel);
      },

      showGameOver: function(levelName, time)
      {
         //canvas.style['-webkit-filter'] = 'blur(2px)';
         var h1 = gameOver.querySelector('h1');
         h1.innerHTML = 'mission fail';

         var img = gameOver.querySelector('img');
         img.style.display = 'none';

         var timeElapsed = gameOver.querySelector('p');
         timeElapsed.innerHTML = '';

         var nextmap = gameOver.querySelector('#next-map');
         nextmap.style.display = 'none';
         gameOver.style.display = 'block';

         // google analytics
         _gaq.push(['_trackEvent', levelName, 'fail',
                    'time', time/1000]);

      },

      showGameWin: function(levelName, time, stars)
      {
         var h1 = gameOver.querySelector('h1');
         h1.innerHTML = 'mission succes';

         var img = gameOver.querySelector('img');
         img.src = 'img/stars'+stars+'.png';
         img.style.display = 'inline';

         var pad = function (n) {
            if ( n > 9 ) return n;
            else         return '0'+n;
         };

         var min = Math.floor(time/(60*1000));
         var sec = Math.floor((time - min * 60*1000)/1000);
         var timeElapsed = gameOver.querySelector('p');
         timeElapsed.innerHTML = 'Time: '+ pad(min) //minutes
                                         + ':'
                                         + pad(sec)      //secodns
                                         + '.'
                                         + time % 1000;   // ms

         var nextmap = gameOver.querySelector('#next-map');
         nextmap.onclick = function() {
            gameOver.style.display = 'none';
            Ses.Engine.nextMap();
         };
         nextmap.style.display = 'block';
         gameOver.style.display = 'block';

         // google analytics
         _gaq.push(['_trackEvent', levelName, 'win',
                    'stars', stars]);
         _gaq.push(['_trackEvent', levelName, 'win',
                    'time', time/1000]);
      }

   };

})();
