# alterego_webapp

```bash
a) Prerequisites
sudo apt update
sudo apt install nodejs npm
sudo npm install -g npm@latest
```

```bash
b) Usando Create React App
npx create-react-app webapp
cd webapp
npm start
```
b-1) Usando Vite (più moderno e veloce):
npm create vite@latest webapp -- --template react --use-npm --legacy-peer-deps
cd webapp
npm install
npm run dev

c)installare alcune dipendenze
npm install lucide-react
npm install tailwindcss postcss autoprefixer
npx tailwindcss init -p



# Come funziona:
webapp/
├── src/
│   ├── components/     # Crea questa cartella
│   ├── App.jsx
│   ├── main.jsx
│   └── index.css