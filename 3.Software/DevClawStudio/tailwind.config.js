/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx}'],
  darkMode: 'class',
  theme: {
    extend: {
      colors: {
        brand: {
          50: '#eef7ff',
          100: '#d9ecff',
          200: '#bcdfff',
          300: '#8ecbff',
          400: '#59adff',
          500: '#338bff',
          600: '#1b6bf5',
          700: '#1455e1',
          800: '#1745b6',
          900: '#193d8f',
        }
      }
    },
  },
  plugins: [],
};
