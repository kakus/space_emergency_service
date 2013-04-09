Ses.Menu = (function() {

   var levelChoose = document.getElementById('level-choose'),
       gameOver = document.getElementById('game-end'),
       canvas   = document.getElementById('canvas'),
       backToMenu = document.getElementById('back-to-menu');

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
         gameOver.style.display = 'block';
      },

      showGameWin: function()
      {
         var h1 = gameOver.querySelector('h1');
         h1.innerHTML = 'mission succes';
         gameOver.style.display = 'block';
      }

   };

})();
