document.addEventListener('DOMContentLoaded', function() {
    // Light/dark theme toggle — flips data-theme on <html> and remembers the
    // choice; a matching inline snippet in <head> applies it again on the next
    // visit before anything paints, so there's no flash of the wrong theme.
    const themeToggle = document.getElementById('themeToggle');
    if (themeToggle) {
        themeToggle.addEventListener('click', function() {
            const root = document.documentElement;
            const isLight = root.getAttribute('data-theme') === 'light';
            if (isLight) { root.removeAttribute('data-theme'); } else { root.setAttribute('data-theme', 'light'); }
            try { localStorage.setItem('theme', isLight ? 'dark' : 'light'); } catch (e) {}
        });
    }

    const menuToggle = document.querySelector('.menu-toggle');
    const mobileNav = document.querySelector('.mobile-nav');

    menuToggle.addEventListener('click', function() {
        menuToggle.classList.toggle('active');
        mobileNav.classList.toggle('active');
    });

    // Close menu when clicking outside
    document.addEventListener('click', function(event) {
        if (!event.target.closest('nav')) {
            menuToggle.classList.remove('active');
            mobileNav.classList.remove('active');
        }
    });
});
