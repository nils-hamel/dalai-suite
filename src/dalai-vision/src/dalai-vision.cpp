/*
 *  dalai-suite - vision
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2017 EPFL CDH DHLAB
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

    # include "dalai-vision.hpp"

/*
    source - constructor/destructor methods
 */

    dl_vision_t::dl_vision_t()

        : vs_window( NULL )
        , vs_context( 0 )
        , vs_width( 0 )
        , vs_height( 0 )
        , vs_near( 0.001 )
        , vs_far( 100.0 )
        , vs_state( true )
        , vs_angle_x( 0.0 )
        , vs_angle_y( 0.0 )
        , vs_angle_z( 0.0 )
        , vs_trans_x( 0.0 )
        , vs_trans_y( 0.0 )
        , vs_trans_z( 0.0 )
        , vs_mouse_x( 0 )
        , vs_mouse_y( 0 )

    {

        /* diplay mode variables */
        SDL_DisplayMode dl_display;

        /* initialise sdl */
        if ( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {

            /* throw exception */
            throw( DL_ERROR_RENDER );

        }

        /* retrieve current display mode */
        if ( SDL_GetCurrentDisplayMode( 0, & dl_display ) < 0 ) {

            /* throw exception */
            throw( DL_ERROR_RENDER );

        }

        /* enable double-buffering */
        SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );

        /* define frame buffer components size */
        SDL_GL_SetAttribute( SDL_GL_RED_SIZE  , 8 );
        SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
        SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE , 8 );
        SDL_GL_SetAttribute( SDL_GL_ALPHA_SIZE, 8 );

        /* define depth buffer size */
        SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 24 );

        /* create sdl window */
        if ( ( vs_window = SDL_CreateWindow( "", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, dl_display.w, dl_display.h, SDL_WINDOW_FULLSCREEN | SDL_WINDOW_OPENGL ) ) == NULL ) {

            /* throw exception */
            throw( DL_ERROR_WINDOW );

        }

        /* create opengl context */
        if ( ( vs_context = SDL_GL_CreateContext( vs_window ) ) == NULL ) {

            /* throw exception */
            throw( DL_ERROR_OPENGL );

        }

        /* opengl clear color */
        glClearColor( 0.00, 0.02, 0.05, 0.0 );

        /* opengl clear depth */
        glClearDepth( 1.0 );

        /* opengl states */
        glEnable( GL_DEPTH_TEST );

        /* opengl blending function */
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

        /* opengl client array */
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_COLOR_ARRAY  );

        /* assign screen dimension */
        vs_width  = dl_display.w;
        vs_height = dl_display.h;

        /* compute projection matrix */
        //vs_set_projection();

    }

    dl_vision_t::~dl_vision_t() {

        /* unknown necessity */
        SDL_Delay( 500 );

        /* delete opengl context */
        SDL_GL_DeleteContext( vs_context );

        /* delete window */
        SDL_DestroyWindow( vs_window );

        /* terminate sdl */
        SDL_Quit();

    }

/*
   source - mutator methods
 */

    void dl_vision_t::vs_set_projection( dl_model_t & dl_model ) {

        /* model minimum mean variables */
        double dl_mean( dl_model.ml_get_mean() );

        /* model wideness variables */
        double dl_wideness( dl_model.ml_get_wideness() );

        /* opengl viewport */
        glViewport( 0.0, 0.0, vs_width, vs_height );

        /* push viewport vector */
        glGetIntegerv( GL_VIEWPORT, vs_viewport );

        /* opengl matrix mode */
        glMatrixMode( GL_PROJECTION );

        /* reset matrix coefficients */
        glLoadIdentity();

        /* compute matrix coefficients */
        gluPerspective( 45.0, double( vs_width ) / double( vs_height ), dl_mean, dl_wideness );

        /* push projection matrix */
        glGetDoublev( GL_PROJECTION_MATRIX, vs_projection );

    }

/*
    source - execution methods
 */

    void dl_vision_t::vs_execution( dl_model_t & dl_model ) {

        /* principale execution loop */
        while ( vs_state == true ) {

            /* events management */
            while ( SDL_PollEvent( & vs_event ) > 0 ) {

                /* switch on event type */
                switch ( vs_event.type ) {

                    /* event : keydown */
                    case ( SDL_KEYDOWN ) : {

                        /* specialised method */
                        vs_keydown( vs_event.key, dl_model );

                    } break;

                    /* event : mouse button */
                    case ( SDL_MOUSEBUTTONDOWN ) : {

                        /* specialised method */
                        vs_button( vs_event.button, dl_model );

                    } break;

                    /* event : mouse motion */
                    case ( SDL_MOUSEMOTION ) : {

                        /* specialised method */
                        vs_motion( vs_event.motion, dl_model );

                    } break;

                    /* event : mouse wheele */
                    case ( SDL_MOUSEWHEEL ) : {

                        /* specialised method */
                        vs_wheel( vs_event.wheel, dl_model );

                    } break;

                };

            }

            /* opengl buffer clear */
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

            /* opengl matrix mode */
            glMatrixMode( GL_MODELVIEW );

            /* reset matrix coefficients */
            glLoadIdentity();

            /* push matrix */
            glPushMatrix();

                /* view translations */
                glTranslatef( vs_trans_x, vs_trans_y, vs_trans_z );

                /* view rotations */
                glRotatef( vs_angle_x, 1.0, 0.0, 0.0 );
                glRotatef( vs_angle_y, 0.0, 1.0, 0.0 );
                glRotatef( vs_angle_z, 0.0, 0.0, 1.0 );

                /* push matrix */
                glPushMatrix();

                    /* display frame */
                    dl_model.ml_ren_frame();

                /* pop matrix */
                glPopMatrix();

                /* push matrix */
                glPushMatrix();

                    /* push matrix */
                    glPushMatrix();

                        /* diplay model */
                        dl_model.ml_ren_model();

                        /* retrieve modelview matrix */
                        glGetDoublev( GL_MODELVIEW_MATRIX, vs_modelview );

                    /* pop matrix */
                    glPopMatrix();

                    /* push matrix */
                    glPushMatrix();

                        /* display surface */
                        dl_model.ml_ren_surface();

                    /* pop matrix */
                    glPopMatrix();

                /* pop matrix */
                glPopMatrix();

            /* pop matrix */
            glPopMatrix();

            /* swap buffers */
            SDL_GL_SwapWindow( vs_window );

        }

    }

/*
    source - event methods
 */

    void dl_vision_t::vs_keydown( SDL_KeyboardEvent vs_event, dl_model_t & dl_model ) {

        /* switch on keycode */
        switch ( vs_event.keysym.sym ) {

            /* keycode : [escape] */
            case ( SDLK_ESCAPE ) : {

                /* update state */
                vs_state = false;

            } break;

            case ( SDLK_1 ) :
            case ( SDLK_2 ) :
            case ( SDLK_3 ) :
            case ( SDLK_4 ) : {

                /* update model point size */
                dl_model.ml_set_wide( vs_event.keysym.sym - SDLK_1 + 1 );

            } break;

            case ( SDLK_TAB ) : {

                /* switch display flag */
                dl_model.ml_set_switch();

            } break;

            case ( SDLK_RETURN ) : {

                /* compute and display intersection */
                dl_model.ml_get_intersect();

            } break;

            case ( SDLK_SPACE ) : {

                /* push active element */
                dl_model.ml_set_push();

            } break;

            case ( SDLK_BACKSPACE ) : {

                /* clear points */
                dl_model.ml_set_clear();

            } break;

            case ( SDLK_a ) : {

                // **
                dl_model.ml_set_autopoint();

            } break;

            case ( SDLK_q ) : {

                // compute plane
                dl_model.ml_set_active( 0 );

            } break;

            case ( SDLK_w ) : {

                // compute plane
                dl_model.ml_set_active( 1 );

            } break;

            case ( SDLK_e ) : {

                // compute plane
                dl_model.ml_set_active( 2 );

            } break;

        };

    }

    void dl_vision_t::vs_button( SDL_MouseButtonEvent vs_event, dl_model_t & dl_model ) {

        /* interaction coordinates variables */
        GLdouble dl_ipx( 0.0 ), dl_ipy( 0.0 ), dl_ipz( 0.0 );

        /* depth component variables */
        GLfloat dl_depth( 0.0 );

        /* assign mouse click position */
        vs_mouse_x = vs_event.x;
        vs_mouse_y = vs_event.y;

        /* check event type */
        if ( vs_event.clicks == 2 ) {

            /* retrieve click depth component */
            glReadPixels( vs_event.x, vs_height - vs_event.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, & dl_depth );

            /* check depth clear value */
            if ( dl_depth < 1.0 ) {

                /* retrieve model click positon */
                gluUnProject( vs_event.x, vs_height - vs_event.y, dl_depth, vs_modelview, vs_projection, vs_viewport, & dl_ipx, & dl_ipy, & dl_ipz );

                /* assign model new pseudo-center */
                dl_model.ml_set_center( dl_ipx, dl_ipy, dl_ipz );

            }

        }

    }

    void dl_vision_t::vs_motion( SDL_MouseMotionEvent vs_event, dl_model_t & dl_model ) {

        /* motion modifier variables */
        float dl_factor( 1.0 );

        /* keyboard modifiers variables */
        SDL_Keymod dl_modifs( SDL_GetModState() );

        /* analyse keyboard modifiers */
        if ( ( dl_modifs & KMOD_CTRL  ) != 0 ) dl_factor = 5.0;
        if ( ( dl_modifs & KMOD_SHIFT ) != 0 ) dl_factor = 0.2;

        /* check state - active buttion */
        if ( ( vs_event.state & SDL_BUTTON_LMASK ) != 0 ) {

            /* update view angles */
            vs_angle_x += ( float ) ( ( int ) vs_event.y - vs_mouse_y ) * DL_INERTIA_ANGLE * dl_factor;

            if ( vs_modelview[10] < 0.0 ) {

                /* update view angles */
                vs_angle_z -= ( float ) ( ( int ) vs_event.x - vs_mouse_x ) * DL_INERTIA_ANGLE * dl_factor;

            } else {

                /* update view angles */
                vs_angle_z += ( float ) ( ( int ) vs_event.x - vs_mouse_x ) * DL_INERTIA_ANGLE * dl_factor;

            }

        } else if ( ( vs_event.state & SDL_BUTTON_RMASK ) != 0 ) {

            /* update view translation */
            vs_trans_x += dl_model.ml_get_wideness() * ( float ) ( ( int ) vs_event.x - vs_mouse_x ) * DL_INERTIA_TRANS * dl_factor;
            vs_trans_y -= dl_model.ml_get_wideness() * ( float ) ( ( int ) vs_event.y - vs_mouse_y ) * DL_INERTIA_TRANS * dl_factor;

        }

    }

    void dl_vision_t::vs_wheel( SDL_MouseWheelEvent vs_event, dl_model_t & dl_model ) {

        /* motion modifier variables */
        float dl_factor( 1.0 );

        /* keyboard modifiers variables */
        SDL_Keymod dl_modifs( SDL_GetModState() );

        /* analyse keyboard modifiers */
        if ( ( dl_modifs & KMOD_CTRL  ) != 0 ) dl_factor = 5.0;
        if ( ( dl_modifs & KMOD_SHIFT ) != 0 ) dl_factor = 0.2;

        /* switch on direction */
        if ( vs_event.y > 0 ) {

            /* update view translation */
            vs_trans_z += dl_model.ml_get_wideness() * DL_INERTIA_WZOOM * dl_factor;

        } else if ( vs_event.y < 0 ) {

            /* update view translation */
            vs_trans_z -= dl_model.ml_get_wideness() * DL_INERTIA_WZOOM * dl_factor;

        }

    }

/*
    source - main function
 */

    int main( int argc, char ** argv ) {

        try {

            /* vision class variables */
            dl_vision_t dl_vision;

            /* model class variables */
            dl_model_t dl_model( argv[1] );

            /* set projection matrix */
            dl_vision.vs_set_projection( dl_model );

            /* principale execution loop */
            dl_vision.vs_execution( dl_model );

        } catch ( int dl_code ) {

            /* display format */
            std::cerr << "dalai-suite : error : ";

            /* switch on error code */
            switch ( dl_code ) {

                case ( DL_ERROR_MEMORY    ) : {

                    /* display message */
                    std::cerr << "memory allocation";

                } break;

                case ( DL_ERROR_IO_ACCESS ) : {

                    /* display message */
                    std::cerr << "unable to access stream";

                } break;

                case ( DL_ERROR_IO_READ   ) : {

                    /* display message */
                    std::cerr << "stream reading";

                } break;

                case ( DL_ERROR_PARAMS    ) : {

                    /* display message */
                    std::cerr << "parameter out of range";

                } break;

                case ( DL_ERROR_RENDER ) : {

                    /* display message */
                    std::cerr << "unable to create rendering context";

                } break;

                case ( DL_ERROR_WINDOW ) : {

                    /* display message */
                    std::cerr << "unable to create rendering window";

                } break;

                case ( DL_ERROR_OPENGL ) : {

                    /* display message */
                    std::cerr << "unable to create opengl context";

                } break;

            };

            /* display format */
            std::cerr << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        } /* otherwise */ {

            /* send message */
            return( EXIT_SUCCESS );

        }

    }

