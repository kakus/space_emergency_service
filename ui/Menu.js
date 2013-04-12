Ses.Menu = (function() {

   var levelChoose = document.getElementById('level-choose'),
       gameOver = document.getElementById('game-end'),
       canvas   = document.getElementById('canvas'),
       restartButton = document.getElementById('restart'),
       backToMenu = document.getElementById('back-to-menu');

   restartButton.onclick = function() {
      gameOver.style.display = 'none';
      Ses.Engine.restartMap();
   };

   backToMenu.onclick = function()
   {
      gameOver.style.display = 'none';
      levelChoose.style.display = 'block';
   };

   return {

      addMap: function(mapName, callback)
      {
         var mapLabel = document.createElement('div');
         mapLabel.className = 'ui button';
         mapLabel.innerHTML = mapName;
         mapLabel.onclick = function() {
            levelChoose.style.display = 'none';
            callback.call();
         };

         levelChoose.appendChild(mapLabel);
      },

      showGameOver: function()
      {
         //canvas.style['-webkit-filter'] = 'blur(2px)';
         var h1 = gameOver.querySelector('h1');
         h1.innerHTML = 'mission fail';

         var img = gameOver.querySelector('img');
         img.style.display = 'none';

         var nextmap = gameOver.querySelector('#next-map');
         nextmap.style.display = 'none';
         gameOver.style.display = 'block';
      },

      showGameWin: function(stars)
      {
         var h1 = gameOver.querySelector('h1');
         h1.innerHTML = 'mission succes';

         var img = gameOver.querySelector('img');
         img.src = 'img/stars'+stars+'.png';
         img.style.display = 'inline';

         var nextmap = gameOver.querySelector('#next-map');
         nextmap.onclick = function() {
            gameOver.style.display = 'none';
            Ses.Engine.nextMap();
         };
         nextmap.style.display = 'block';
         gameOver.style.display = 'block';
      }

   };

})();
