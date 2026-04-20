/* =================================================================
   Ground-Air Autonomous Platform — Specification Corpus
   shared.js · shared client-side behaviors
   Revision 0.1 · April 2026
   ================================================================= */

(function () {
  'use strict';

  // -----------------------------------------------------------------
  // 1. TOC scrollspy — highlights active section in the sidebar
  // -----------------------------------------------------------------
  function initScrollspy() {
    const sections = document.querySelectorAll('section[id]');
    const tocLinks = document.querySelectorAll('.toc-link');
    if (!sections.length || !tocLinks.length) return;

    const observer = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (entry.isIntersecting) {
          const id = entry.target.id;
          tocLinks.forEach(function (link) {
            link.classList.toggle('active', link.getAttribute('href') === '#' + id);
          });
        }
      });
    }, { rootMargin: '-80px 0px -60% 0px', threshold: 0 });

    sections.forEach(function (s) { observer.observe(s); });
  }

  // -----------------------------------------------------------------
  // 2. Copy link (globally exposed as window.copyLink)
  // -----------------------------------------------------------------
  window.copyLink = function (evt) {
    try {
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(window.location.href);
      } else {
        const textarea = document.createElement('textarea');
        textarea.value = window.location.href;
        document.body.appendChild(textarea);
        textarea.select();
        document.execCommand('copy');
        document.body.removeChild(textarea);
      }
      const btn = (evt && evt.target) || document.activeElement;
      if (btn && btn.textContent) {
        const original = btn.textContent;
        btn.textContent = 'COPIED';
        setTimeout(function () { btn.textContent = original; }, 1500);
      }
    } catch (e) {
      /* silently ignore clipboard failures on local file:// origins */
    }
  };

  // -----------------------------------------------------------------
  // 3. Print (exposed as window.printDoc)
  // -----------------------------------------------------------------
  window.printDoc = function () { window.print(); };

  // -----------------------------------------------------------------
  // 4. Node-anatomy pan/zoom + expand-all / collapse-all
  //    Only runs on pages that include a .canvas-wrap element.
  // -----------------------------------------------------------------
  function initAnatomy() {
    const wrap = document.getElementById('canvasWrap');
    const canvas = document.getElementById('canvas');
    if (!wrap || !canvas) return;

    const zoomInd = document.getElementById('zoomIndicator');
    let scale = 0.5;
    let tx = 0, ty = 0;
    const minScale = 0.2, maxScale = 2.5;

    function applyTransform() {
      canvas.style.transform = 'translate(' + tx + 'px, ' + ty + 'px) scale(' + scale + ')';
      if (zoomInd) zoomInd.textContent = Math.round(scale * 100) + '%';
    }

    function centerView() {
      const wrapRect = wrap.getBoundingClientRect();
      const canvasW = canvas.offsetWidth || 3600;
      const canvasH = canvas.offsetHeight || 2400;
      scale = Math.min(wrapRect.width / canvasW, wrapRect.height / canvasH) * 0.95;
      tx = (wrapRect.width - canvasW * scale) / 2;
      ty = (wrapRect.height - canvasH * scale) / 2;
      applyTransform();
    }

    window.addEventListener('load', function () { setTimeout(centerView, 50); });
    window.addEventListener('resize', centerView);

    // Drag to pan
    let isDragging = false, dragStartX, dragStartY, startTx, startTy;
    wrap.addEventListener('mousedown', function (e) {
      if (e.target.closest('.leaf')) return;
      isDragging = true;
      wrap.classList.add('grabbing');
      dragStartX = e.clientX;
      dragStartY = e.clientY;
      startTx = tx; startTy = ty;
      e.preventDefault();
    });
    window.addEventListener('mousemove', function (e) {
      if (!isDragging) return;
      tx = startTx + (e.clientX - dragStartX);
      ty = startTy + (e.clientY - dragStartY);
      applyTransform();
    });
    window.addEventListener('mouseup', function () {
      isDragging = false;
      wrap.classList.remove('grabbing');
    });

    // Scroll to zoom
    wrap.addEventListener('wheel', function (e) {
      e.preventDefault();
      const delta = -e.deltaY * 0.001;
      const newScale = Math.max(minScale, Math.min(maxScale, scale * (1 + delta)));
      const wrapRect = wrap.getBoundingClientRect();
      const mx = e.clientX - wrapRect.left;
      const my = e.clientY - wrapRect.top;
      const ratio = newScale / scale;
      tx = mx - (mx - tx) * ratio;
      ty = my - (my - ty) * ratio;
      scale = newScale;
      applyTransform();
    }, { passive: false });

    // Touch
    let touchStart = null;
    wrap.addEventListener('touchstart', function (e) {
      if (e.touches.length === 1 && !e.target.closest('.leaf')) {
        touchStart = { x: e.touches[0].clientX, y: e.touches[0].clientY, tx: tx, ty: ty };
      } else if (e.touches.length === 2) {
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        touchStart = { pinchDist: Math.sqrt(dx * dx + dy * dy), startScale: scale };
      }
    });
    wrap.addEventListener('touchmove', function (e) {
      if (!touchStart) return;
      e.preventDefault();
      if (e.touches.length === 1 && touchStart.x !== undefined) {
        tx = touchStart.tx + (e.touches[0].clientX - touchStart.x);
        ty = touchStart.ty + (e.touches[0].clientY - touchStart.y);
        applyTransform();
      } else if (e.touches.length === 2 && touchStart.pinchDist) {
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        const d = Math.sqrt(dx * dx + dy * dy);
        scale = Math.max(minScale, Math.min(maxScale, touchStart.startScale * (d / touchStart.pinchDist)));
        applyTransform();
      }
    }, { passive: false });
    wrap.addEventListener('touchend', function () { touchStart = null; });

    function zoomCentered(factor) {
      const wrapRect = wrap.getBoundingClientRect();
      const mx = wrapRect.width / 2, my = wrapRect.height / 2;
      const newScale = Math.max(minScale, Math.min(maxScale, scale * factor));
      const ratio = newScale / scale;
      tx = mx - (mx - tx) * ratio;
      ty = my - (my - ty) * ratio;
      scale = newScale;
      applyTransform();
    }

    const zIn  = document.getElementById('zoomIn');
    const zOut = document.getElementById('zoomOut');
    const zRst = document.getElementById('resetView');
    if (zIn)  zIn.addEventListener('click',  function () { zoomCentered(1.2); });
    if (zOut) zOut.addEventListener('click', function () { zoomCentered(1 / 1.2); });
    if (zRst) zRst.addEventListener('click', centerView);

    // Leaf expand / collapse
    document.querySelectorAll('.leaf').forEach(function (leaf) {
      leaf.addEventListener('click', function (e) {
        e.stopPropagation();
        leaf.classList.toggle('expanded');
      });
    });

    const expandAll = document.getElementById('expandAll');
    const collapseAll = document.getElementById('collapseAll');
    if (expandAll) expandAll.addEventListener('click', function () {
      document.querySelectorAll('.leaf').forEach(function (l) { l.classList.add('expanded'); });
    });
    if (collapseAll) collapseAll.addEventListener('click', function () {
      document.querySelectorAll('.leaf').forEach(function (l) { l.classList.remove('expanded'); });
    });

    // Draw connector lines
    function drawLines() {
      const svg = document.getElementById('lines');
      if (!svg) return;
      svg.innerHTML = '';

      const center = document.getElementById('centerNode');
      if (!center) return;

      const cx = parseFloat(center.style.left || '1560') + (center.offsetWidth || 480) / 2;
      const cy = parseFloat(center.style.top  || '1080') + (center.offsetHeight || 240) / 2;

      const branches = document.querySelectorAll('.branch');
      branches.forEach(function (b) {
        const left = parseFloat(b.style.left);
        const top = parseFloat(b.style.top);
        const width = b.offsetWidth || 340;
        const bx = left + width / 2;
        const by = top + 30;

        const mx = (cx + bx) / 2;
        const my = (cy + by) / 2;

        let color = 'var(--line)';
        if      (b.classList.contains('pub'))      color = 'var(--accent-pub)';
        else if (b.classList.contains('sub'))      color = 'var(--accent-sub)';
        else if (b.classList.contains('srv'))      color = 'var(--accent-srv)';
        else if (b.classList.contains('param'))    color = 'var(--accent-param)';
        else if (b.classList.contains('internal')) color = 'var(--accent-internal)';
        else if (b.classList.contains('safety'))   color = 'var(--accent-safety)';
        else if (b.classList.contains('hw'))       color = 'var(--accent-hw)';

        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('d', 'M ' + cx + ' ' + cy + ' Q ' + mx + ' ' + my + ' ' + bx + ' ' + by);
        path.setAttribute('stroke', color);
        path.setAttribute('stroke-width', '1.5');
        path.setAttribute('fill', 'none');
        path.setAttribute('opacity', '0.35');
        path.setAttribute('stroke-dasharray', '4 4');
        svg.appendChild(path);

        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', bx);
        circle.setAttribute('cy', by);
        circle.setAttribute('r', '4');
        circle.setAttribute('fill', color);
        circle.setAttribute('opacity', '0.7');
        svg.appendChild(circle);
      });
    }

    window.addEventListener('load', function () { setTimeout(drawLines, 100); });
  }

  // -----------------------------------------------------------------
  // 5. Optional client-side filter (index pages)
  // -----------------------------------------------------------------
  function initIndexFilter() {
    const input = document.getElementById('docFilter');
    if (!input) return;
    const entries = document.querySelectorAll('[data-filterable]');
    input.addEventListener('input', function () {
      const q = input.value.trim().toLowerCase();
      entries.forEach(function (el) {
        const hay = (el.getAttribute('data-filterable') || el.textContent).toLowerCase();
        el.style.display = q === '' || hay.indexOf(q) !== -1 ? '' : 'none';
      });
    });
  }

  // -----------------------------------------------------------------
  // Boot
  // -----------------------------------------------------------------
  document.addEventListener('DOMContentLoaded', function () {
    initScrollspy();
    initAnatomy();
    initIndexFilter();
  });
})();
