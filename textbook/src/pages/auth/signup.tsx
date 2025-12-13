import React from 'react';
import Layout from '@theme/Layout';
import { SignupForm } from '../../components/auth/SignupForm';
import { useHistory } from '@docusaurus/router';

export default function SignupPage(): JSX.Element {
  const history = useHistory();

  const handleSignupSuccess = () => {
    // Redirect to home page after successful signup
    history.push('/');
  };

  return (
    <Layout
      title="Sign Up"
      description="Create your account to access personalized learning"
    >
      <main>
        <SignupForm onSuccess={handleSignupSuccess} />
      </main>
    </Layout>
  );
}
