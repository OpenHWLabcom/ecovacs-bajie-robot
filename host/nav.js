/* nav.js — 在所有分页右下/左下角注入「导航」入口；iframe/embed 内不显示 */
(function () {
  try {
    if (window.top !== window) return;
    if (/[?&]embed=1\b/.test(location.search)) return;
    if (document.getElementById('__robotNavHome')) return;
    var a = document.createElement('a');
    a.id = '__robotNavHome';
    a.href = 'robot_home.html';
    a.textContent = '🏠 导航';
    a.title = '机器人控制系统导航总页';
    a.style.cssText = [
      'position:fixed', 'left:14px', 'bottom:14px', 'z-index:2147483000',
      'background:#0b1729', 'border:1px solid #22344d', 'color:#9fd8ff',
      'border-radius:999px', 'padding:7px 12px',
      'font:600 12px ui-monospace,SFMono-Regular,Menlo,monospace',
      'text-decoration:none', 'box-shadow:0 6px 18px #0007',
      'opacity:.82', 'transition:opacity .15s,border-color .15s'
    ].join(';');
    a.onmouseenter = function () { a.style.opacity = '1'; a.style.borderColor = '#22d3ee88'; };
    a.onmouseleave = function () { a.style.opacity = '.82'; a.style.borderColor = '#22344d'; };
    document.body.appendChild(a);
  } catch (e) {}
})();
