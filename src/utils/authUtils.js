// Utility functions for authentication
import { useAuth } from '../components/Auth/AuthProvider';

// Custom hook to access authentication status
export const useAuthStatus = () => {
  const { user, loading, error } = useAuth();
  
  return {
    isAuthenticated: !!user,
    user,
    loading,
    error
  };
};

// Higher-order component to wrap pages that require authentication
export const withAuth = (Component) => {
  return function AuthenticatedComponent(props) {
    const { isAuthenticated, loading, user } = useAuthStatus();
    
    if (loading) {
      return (
        <div style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '100vh',
          fontSize: '18px'
        }}>
          Loading...
        </div>
      );
    }
    
    if (!isAuthenticated) {
      // Redirect to login
      window.location.href = '/auth/login';
      return null;
    }
    
    return <Component {...props} user={user} />;
  };
};