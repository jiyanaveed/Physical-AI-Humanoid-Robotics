import axios, { AxiosInstance, AxiosRequestConfig } from 'axios';

// Get the RAG API URL from environment variables
const ragApiUrl = process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000';
const timeout = parseInt(process.env.REACT_APP_RAG_API_TIMEOUT || '30000', 10);

// Create the API client instance
const ragApiClient: AxiosInstance = axios.create({
  baseURL: ragApiUrl,
  timeout: timeout,
  headers: {
    'Content-Type': 'application/json',
  },
});

export default ragApiClient;